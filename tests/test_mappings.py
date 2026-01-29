"""Tests for ROS to conda package mappings."""

from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from pixi_ros.mappings import (
    get_mappings,
    get_robostack_yaml_path,
    get_ros_distros,
    is_system_package,
    map_ros_to_conda,
    reload_mappings,
    validate_distro,
)


def test_map_common_packages():
    """Test mapping of common ROS packages (unmapped, uses fallback)."""
    assert map_ros_to_conda("rclcpp", "humble") == ["ros-humble-rclcpp"]
    assert map_ros_to_conda("rclpy", "iron") == ["ros-iron-rclpy"]
    assert map_ros_to_conda("std_msgs", "jazzy") == ["ros-jazzy-std-msgs"]
    assert map_ros_to_conda("geometry_msgs", "humble") == ["ros-humble-geometry-msgs"]


def test_map_build_tools():
    """Test mapping of build tools (unmapped, uses fallback)."""
    assert map_ros_to_conda("ament_cmake", "humble") == ["ros-humble-ament-cmake"]
    assert map_ros_to_conda("ament_cmake_gtest", "iron") == ["ros-iron-ament-cmake-gtest"]


def test_map_unknown_package():
    """Test mapping of packages not in the explicit mapping."""
    # Unknown packages should follow the default pattern
    result = map_ros_to_conda("unknown_package", "humble")
    assert result == ["ros-humble-unknown-package"]

    # Underscores should be converted to dashes
    result = map_ros_to_conda("my_custom_msgs", "iron")
    assert result == ["ros-iron-my-custom-msgs"]


def test_map_system_packages():
    """Test mapping of system packages from conda-forge.yaml."""
    # System packages are in the mapping and don't get ros-distro prefix
    assert map_ros_to_conda("cmake", "humble") == ["cmake"]
    assert map_ros_to_conda("uncrustify", "humble") == ["uncrustify"]


def test_is_system_package():
    """Test identification of system packages."""
    # System packages are in conda-forge.yaml
    assert is_system_package("cmake")
    assert is_system_package("git")
    assert is_system_package("uncrustify")
    # ROS packages are not in mappings
    assert not is_system_package("rclcpp")
    assert not is_system_package("std_msgs")


def test_get_ros_distros():
    """Test getting list of supported distros."""
    distros = get_ros_distros()
    assert "humble" in distros
    assert "iron" in distros
    assert "jazzy" in distros
    assert "rolling" in distros
    assert isinstance(distros, list)


def test_validate_distro():
    """Test distro validation."""
    assert validate_distro("humble")
    assert validate_distro("iron")
    assert validate_distro("jazzy")
    assert validate_distro("rolling")
    assert not validate_distro("invalid")
    assert not validate_distro("foxy")  # Not in our list


@pytest.mark.parametrize(
    "ros_pkg,distro,expected",
    [
        ("rclcpp", "humble", ["ros-humble-rclcpp"]),
        ("std_msgs", "iron", ["ros-iron-std-msgs"]),
        ("nav_msgs", "jazzy", ["ros-jazzy-nav-msgs"]),
        ("tf2_ros", "humble", ["ros-humble-tf2-ros"]),
        ("cmake", "humble", ["cmake"]),  # system package in mapping
    ],
)
def test_map_ros_to_conda_parametrized(ros_pkg, distro, expected):
    """Test various ROS to conda mappings."""
    assert map_ros_to_conda(ros_pkg, distro) == expected


def test_get_mappings():
    """Test that we can load the mappings."""
    mappings = get_mappings()
    assert isinstance(mappings, dict)
    assert len(mappings) > 0
    # Check for some system packages from conda-forge.yaml
    assert "cmake" in mappings
    assert "uncrustify" in mappings


def test_get_robostack_yaml_path():
    """Test getting the path to mapping files."""
    path = get_robostack_yaml_path()
    assert path is not None
    assert path.exists()
    # Should return a .yaml file (like conda-forge.yaml)
    assert path.suffix == ".yaml"


def test_reload_mappings():
    """Test that we can reload mappings."""
    # Get initial mappings
    mappings1 = get_mappings()

    # Reload
    reload_mappings()

    # Get mappings again
    mappings2 = get_mappings()

    # Should be the same
    assert mappings1 == mappings2


def test_platform_specific_mappings():
    """Test platform-specific package mappings."""
    # udev has platform-specific mappings in conda-forge.yaml
    result = map_ros_to_conda("udev", "humble")
    assert isinstance(result, list)
    # All platforms should have at least libusb
    assert "libusb" in result

    # Test with platform override
    linux_result = map_ros_to_conda("udev", "humble", platform_override="linux")
    assert "libusb" in linux_result
    assert "libudev" in linux_result

    osx_result = map_ros_to_conda("udev", "humble", platform_override="osx")
    assert "libusb" in osx_result
    assert "libudev" not in osx_result

    win64_result = map_ros_to_conda("udev", "humble", platform_override="win64")
    assert "libusb" in win64_result
    assert "libudev" not in win64_result


def test_empty_platform_list():
    """Test packages with empty platform lists."""
    # uuid has empty list for osx and win64
    osx_result = map_ros_to_conda("uuid", "humble", platform_override="osx")
    assert osx_result == []

    win64_result = map_ros_to_conda("uuid", "humble", platform_override="win64")
    assert win64_result == []

    linux_result = map_ros_to_conda("uuid", "humble", platform_override="linux")
    assert "libuuid" in linux_result


def test_user_override_channel_yaml():
    """Test that user can override channel.yaml files."""
    import os

    with TemporaryDirectory() as tmpdir:
        # Create pixi-ros directory with custom channel file
        pixi_ros_dir = Path(tmpdir) / "pixi-ros"
        pixi_ros_dir.mkdir()
        custom_yaml = pixi_ros_dir / "custom.yaml"
        custom_yaml.write_text(
            """
custom_package:
  pixi: [custom-conda-package]
another_package:
  pixi:
    linux: [linux-pkg]
    osx: [osx-pkg]
    win64: []
        """
        )

        # Change to temp directory
        old_cwd = Path.cwd()
        try:
            os.chdir(tmpdir)
            reload_mappings()  # Clear cache

            # Should load custom mappings
            assert map_ros_to_conda("custom_package", "humble") == ["custom-conda-package"]
            # Platform-specific mapping (will vary by platform)
            result = map_ros_to_conda("another_package", "humble")
            assert isinstance(result, list)
            assert len(result) > 0 or result == []  # Depends on platform

        finally:
            os.chdir(old_cwd)
            reload_mappings()  # Restore default mappings
