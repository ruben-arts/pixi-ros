"""Tests for ROS package validation."""

from unittest.mock import MagicMock, patch

import pytest

from pixi_ros.validator import (
    PackageSource,
    PackageValidationResult,
    RosDistroValidator,
)


@pytest.fixture
def mock_validator():
    """Create a mock validator with stubbed distro data."""
    with patch("pixi_ros.validator.get_index"), patch(
        "pixi_ros.validator.get_cached_distribution"
    ) as mock_distro:
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
        return validator


def test_validator_initialization():
    """Test that validator initializes with rosdistro."""
    with patch("pixi_ros.validator.get_index"), patch(
        "pixi_ros.validator.get_cached_distribution"
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

    result = mock_validator.validate_package(
        "my_package", workspace_packages, mappings
    )

    assert result.package_name == "my_package"
    assert result.source == PackageSource.WORKSPACE
    assert result.conda_packages == []
    assert result.error is None


def test_validate_mapped_package_simple(mock_validator):
    """Test validation of mapped packages (simple list)."""
    workspace_packages = set()
    mappings = {
        "cmake": {
            "pixi": ["cmake"]
        }
    }

    result = mock_validator.validate_package(
        "cmake", workspace_packages, mappings
    )

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

    result = mock_validator.validate_package(
        "rclcpp", workspace_packages, mappings
    )

    assert result.package_name == "rclcpp"
    assert result.source == PackageSource.ROS_DISTRO
    assert result.conda_packages == ["ros-humble-rclcpp"]
    assert result.error is None


def test_validate_ros_distro_package_with_underscores(mock_validator):
    """Test validation of ROS packages with underscores."""
    workspace_packages = set()
    mappings = {}

    result = mock_validator.validate_package(
        "std_msgs", workspace_packages, mappings
    )

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
        result = mock_validator.validate_package(
            "eigen", workspace_packages, mappings
        )

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
    mappings = {
        "test_pkg": {
            "pixi": ["mapped-pkg"]
        }
    }

    # Even though it's in mappings and might be in ROS distro,
    # workspace packages take priority
    result = mock_validator.validate_package(
        "test_pkg", workspace_packages, mappings
    )

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
    mappings = {
        "empty_pkg": {}
    }

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
    mappings = {
        "cmake": {"pixi": ["cmake"]}
    }

    results = []
    for pkg in ["ws_pkg", "cmake", "rclcpp", "unknown"]:
        with patch.object(
            mock_validator, "check_conda_forge_availability", return_value=False
        ):
            result = mock_validator.validate_package(
                pkg, workspace_packages, mappings
            )
            results.append(result)

    assert results[0].source == PackageSource.WORKSPACE
    assert results[1].source == PackageSource.MAPPING
    assert results[2].source == PackageSource.ROS_DISTRO
    assert results[3].source == PackageSource.NOT_FOUND
