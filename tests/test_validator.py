"""Tests for ROS package validation."""

from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from pixi_ros.validator import (
    PackageSource,
    RosDistroValidator,
)


@pytest.fixture
def mock_validator():
    """Create a mock validator with stubbed distro data."""
    with (
        patch("pixi_ros.validator.get_index"),
        patch("pixi_ros.validator.get_cached_distribution") as mock_distro,
    ):
        # Mock the distribution to have a few packages
        mock_dist = MagicMock()
        mock_dist.release_packages = {
            "rclcpp": {},
            "rclpy": {},
            "std_msgs": {},
            "sensor_msgs": {},
        }
        mock_distro.return_value = mock_dist

        validator = RosDistroValidator("humble")

        # Mock check_package_availability to return True for ROS packages
        def mock_check_availability(package_name, platform, channel_url=None):
            # ROS packages (with ros-distro- prefix) are available
            if package_name.startswith("ros-humble-"):
                return True
            # Some conda-forge packages are available (used in tests)
            if package_name in [
                "eigen",
                "cmake",
                "boost",
                "libusb",
                "libudev",
                "qt5",
            ]:
                return True
            return False

        validator.check_package_availability = MagicMock(
            side_effect=mock_check_availability
        )
        validator.check_conda_forge_availability = MagicMock(
            side_effect=mock_check_availability
        )

        return validator


def test_validator_initialization():
    """Test that validator initializes with rosdistro."""
    with (
        patch("pixi_ros.validator.get_index"),
        patch("pixi_ros.validator.get_cached_distribution"),
    ):
        validator = RosDistroValidator("humble")
        assert validator.distro_name == "humble"


def test_validator_initialization_error():
    """Test that validator handles rosdistro initialization errors."""
    with patch("pixi_ros.validator.get_index", side_effect=Exception("Network error")):
        validator = RosDistroValidator("humble")
        assert validator._init_error is not None
        assert validator._distro is None


def test_has_package_success(mock_validator):
    """Test checking if package exists in ROS distro."""
    assert mock_validator.has_package("rclcpp")
    assert mock_validator.has_package("std_msgs")
    assert not mock_validator.has_package("unknown_package")


def test_has_package_when_distro_not_loaded():
    """Test has_package returns False when distro failed to load."""
    with patch("pixi_ros.validator.get_index", side_effect=Exception("Network error")):
        validator = RosDistroValidator("humble")
        assert not validator.has_package("rclcpp")


def test_validate_workspace_package(mock_validator):
    """Test validation of workspace packages."""
    workspace_packages = {"my_package", "another_package"}
    mappings = {}

    result = mock_validator.validate_package("my_package", workspace_packages, mappings)

    assert result.package_name == "my_package"
    assert result.source == PackageSource.WORKSPACE
    assert result.conda_packages == []
    assert result.error is None


def test_validate_mapped_package_simple(mock_validator):
    """Test validation of mapped packages (simple list)."""
    workspace_packages = set()
    mappings = {"cmake": {"pixi": ["cmake"]}}

    result = mock_validator.validate_package("cmake", workspace_packages, mappings)

    assert result.package_name == "cmake"
    assert result.source == PackageSource.MAPPING
    assert result.conda_packages == ["cmake"]
    assert result.error is None


def test_validate_mapped_package_platform_specific(mock_validator):
    """Test validation of mapped packages with platform-specific mappings."""
    workspace_packages = set()
    mappings = {
        "udev": {
            "pixi": {
                "linux": ["libusb", "libudev"],
                "osx": ["libusb"],
                "win64": ["libusb"],
            }
        }
    }

    # Test linux platform
    result = mock_validator.validate_package(
        "udev", workspace_packages, mappings, platform="linux-64"
    )

    assert result.package_name == "udev"
    assert result.source == PackageSource.MAPPING
    assert result.conda_packages == ["libusb", "libudev"]
    assert result.error is None

    # Test macOS platform
    result = mock_validator.validate_package(
        "udev", workspace_packages, mappings, platform="osx-arm64"
    )

    assert result.package_name == "udev"
    assert result.source == PackageSource.MAPPING
    assert result.conda_packages == ["libusb"]
    assert result.error is None


def test_validate_ros_distro_package(mock_validator):
    """Test validation of packages from ROS distro."""
    workspace_packages = set()
    mappings = {}

    result = mock_validator.validate_package("rclcpp", workspace_packages, mappings)

    assert result.package_name == "rclcpp"
    assert result.source == PackageSource.ROS_DISTRO
    assert result.conda_packages == ["ros-humble-rclcpp"]
    assert result.error is None


def test_validate_ros_distro_package_with_underscores(mock_validator):
    """Test validation of ROS packages with underscores."""
    workspace_packages = set()
    mappings = {}

    result = mock_validator.validate_package("std_msgs", workspace_packages, mappings)

    assert result.package_name == "std_msgs"
    assert result.source == PackageSource.ROS_DISTRO
    assert result.conda_packages == ["ros-humble-std-msgs"]
    assert result.error is None


def test_validate_conda_forge_package(mock_validator):
    """Test validation of packages from conda-forge."""
    workspace_packages = set()
    mappings = {}

    # Mock the conda-forge availability check
    with patch.object(
        mock_validator, "check_conda_forge_availability", return_value=True
    ):
        result = mock_validator.validate_package("eigen", workspace_packages, mappings)

        assert result.package_name == "eigen"
        assert result.source == PackageSource.CONDA_FORGE
        assert result.conda_packages == ["eigen"]
        assert result.error is None


def test_validate_not_found_package(mock_validator):
    """Test validation of packages not found anywhere."""
    workspace_packages = set()
    mappings = {}

    # Mock the conda-forge availability check to return False
    with patch.object(
        mock_validator, "check_conda_forge_availability", return_value=False
    ):
        result = mock_validator.validate_package(
            "unknown_pkg", workspace_packages, mappings
        )

        assert result.package_name == "unknown_pkg"
        assert result.source == PackageSource.NOT_FOUND
        assert result.conda_packages == []
        assert result.error is not None
        assert "not found" in result.error.lower()


def test_validation_priority_order(mock_validator):
    """Test that validation follows the correct priority order."""
    workspace_packages = {"test_pkg"}
    mappings = {"test_pkg": {"pixi": ["mapped-pkg"]}}

    # Even though it's in mappings and might be in ROS distro,
    # workspace packages take priority
    result = mock_validator.validate_package("test_pkg", workspace_packages, mappings)

    assert result.source == PackageSource.WORKSPACE


def test_check_conda_forge_availability_timeout(mock_validator):
    """Test that conda-forge check handles timeouts gracefully."""
    import asyncio

    with patch("pixi_ros.validator.Gateway") as mock_gateway:
        mock_gateway.return_value.query.side_effect = asyncio.TimeoutError()

        result = mock_validator.check_conda_forge_availability("test", "linux-64")
        assert result is False


def test_check_conda_forge_availability_error(mock_validator):
    """Test that conda-forge check handles errors gracefully."""
    with patch("pixi_ros.validator.Gateway", side_effect=Exception("Network error")):
        result = mock_validator.check_conda_forge_availability("test", "linux-64")
        assert result is False


def test_validate_package_handles_empty_mapping(mock_validator):
    """Test validation when mapping exists but is empty."""
    workspace_packages = set()
    mappings = {"empty_pkg": {}}

    # Should fall through to ROS distro check or conda-forge
    with patch.object(
        mock_validator, "check_conda_forge_availability", return_value=False
    ):
        result = mock_validator.validate_package(
            "empty_pkg", workspace_packages, mappings
        )

        # Since it's not in ROS distro (mock only has rclcpp, std_msgs, etc),
        # it should be NOT_FOUND
        assert result.source == PackageSource.NOT_FOUND


def test_validate_multiple_packages(mock_validator):
    """Test validating multiple packages in sequence."""
    workspace_packages = {"ws_pkg"}
    mappings = {"cmake": {"pixi": ["cmake"]}}

    # Test without mocking availability (using default mock from fixture)
    results = []
    for pkg in ["ws_pkg", "cmake", "rclcpp", "unknown"]:
        result = mock_validator.validate_package(pkg, workspace_packages, mappings)
        results.append(result)

    assert results[0].source == PackageSource.WORKSPACE
    assert results[1].source == PackageSource.MAPPING
    assert results[2].source == PackageSource.ROS_DISTRO
    assert results[3].source == PackageSource.NOT_FOUND


def test_validate_ros_distro_package_not_available_in_channels(mock_validator):
    """Test ROS distro package that is not available in channels."""
    workspace_packages = set()
    mappings = {}

    # Mock check_package_availability to return False for this package
    with patch.object(mock_validator, "check_package_availability", return_value=False):
        result = mock_validator.validate_package("rclcpp", workspace_packages, mappings)

    # Package should be marked as NOT_FOUND but include the ROS package name
    assert result.package_name == "rclcpp"
    assert result.source == PackageSource.NOT_FOUND
    assert result.conda_packages == ["ros-humble-rclcpp"]
    assert "ROS package not available" in result.error
    assert "robostack-humble" in result.error


# ---------------------------------------------------------------------------
# Custom channel tests (use local test channel, no network required)
# ---------------------------------------------------------------------------

LOCAL_TEST_CHANNEL = Path(__file__).parent / "data" / "test_channel"


@pytest.fixture
def local_channel_validator():
    """Validator that has no rosdistro packages and points at the local test channel."""
    with (
        patch("pixi_ros.validator.get_index"),
        patch("pixi_ros.validator.get_cached_distribution") as mock_distro,
    ):
        mock_dist = MagicMock()
        mock_dist.release_packages = {}
        mock_distro.return_value = mock_dist

        validator = RosDistroValidator(
            "kilted",
            extra_channels=[LOCAL_TEST_CHANNEL.as_uri()],
        )
        # Ensure conda-forge returns False so we fall through to the custom channel step
        validator.check_conda_forge_availability = MagicMock(return_value=False)
        return validator


def test_custom_channel_finds_package_with_ros_prefix(local_channel_validator):
    """custom_testing_package should be resolved as ros-kilted-custom-testing-package
    from the local test channel."""
    result = local_channel_validator.validate_package(
        "custom_testing_package", set(), {}, platform="linux-64"
    )

    assert result.source == PackageSource.CUSTOM_CHANNEL
    assert result.conda_packages == ["ros-kilted-custom-testing-package"]
    assert result.error is None


def test_custom_channel_stores_channel_url(local_channel_validator):
    """The result's channel field should contain the URL of the channel where the
    package was found."""
    result = local_channel_validator.validate_package(
        "custom_testing_package", set(), {}, platform="linux-64"
    )

    assert result.channel == LOCAL_TEST_CHANNEL.as_uri()


def test_custom_channel_not_found_returns_not_found(local_channel_validator):
    """Packages absent from all sources including custom channels return NOT_FOUND
    with channel=None."""
    result = local_channel_validator.validate_package(
        "nonexistent_package", set(), {}, platform="linux-64"
    )

    assert result.source == PackageSource.NOT_FOUND
    assert result.channel is None


def test_custom_channel_comes_after_conda_forge():
    """When a package is available on conda-forge it should be resolved there,
    not from the custom channel."""
    with (
        patch("pixi_ros.validator.get_index"),
        patch("pixi_ros.validator.get_cached_distribution") as mock_distro,
    ):
        mock_dist = MagicMock()
        mock_dist.release_packages = {}
        mock_distro.return_value = mock_dist

        validator = RosDistroValidator(
            "kilted",
            extra_channels=[LOCAL_TEST_CHANNEL.as_uri()],
        )
        validator.check_conda_forge_availability = MagicMock(return_value=True)

        result = validator.validate_package(
            "custom_testing_package", set(), {}, platform="linux-64"
        )

    assert result.source == PackageSource.CONDA_FORGE
    assert result.channel is None


def test_custom_channel_relative_path_resolves(tmp_path, monkeypatch):
    """Passing a relative path like ./tests/data/test_channel should resolve to a
    file:// URI and still find packages in the local channel."""
    from pixi_ros.init import init_workspace

    # Change cwd to repo root so the relative path resolves correctly
    repo_root = LOCAL_TEST_CHANNEL.parent.parent.parent
    monkeypatch.chdir(repo_root)

    relative = "./tests/data/test_channel"
    resolved_uri = LOCAL_TEST_CHANNEL.resolve().as_uri()

    # Build a minimal workspace in tmp_path so init_workspace has something to work with
    src = tmp_path / "src" / "my_pkg"
    src.mkdir(parents=True)
    (src / "package.xml").write_text(
        '<?xml version="1.0"?>'
        '<package format="3"><name>my_pkg</name><version>0.0.1</version>'
        "<description>test</description>"
        '<maintainer email="a@b.com">A</maintainer>'
        "<license>MIT</license>"
        "<depend>custom_testing_package</depend>"
        "</package>"
    )

    with (
        patch("pixi_ros.validator.get_index"),
        patch("pixi_ros.validator.get_cached_distribution") as mock_distro,
        patch(
            "pixi_ros.validator.RosDistroValidator.check_conda_forge_availability",
            return_value=False,
        ),
    ):
        mock_dist = MagicMock()
        mock_dist.release_packages = {}
        mock_distro.return_value = mock_dist

        init_workspace("kilted", workspace_path=tmp_path, channels=[relative])

    pixi_toml = (tmp_path / "pixi.toml").read_text()
    # The original relative path should be preserved in pixi.toml (pixi handles it
    # natively); only the validator needs the resolved file:// URI.
    assert relative in pixi_toml
    assert resolved_uri not in pixi_toml


def test_custom_channel_skipped_when_no_extra_channels():
    """With no extra channels configured the validator should return NOT_FOUND
    rather than checking any custom channel."""
    with (
        patch("pixi_ros.validator.get_index"),
        patch("pixi_ros.validator.get_cached_distribution") as mock_distro,
    ):
        mock_dist = MagicMock()
        mock_dist.release_packages = {}
        mock_distro.return_value = mock_dist

        validator = RosDistroValidator("kilted")  # no extra_channels
        validator.check_conda_forge_availability = MagicMock(return_value=False)

        result = validator.validate_package(
            "custom_testing_package", set(), {}, platform="linux-64"
        )

    assert result.source == PackageSource.NOT_FOUND
