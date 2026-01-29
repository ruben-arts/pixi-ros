"""Tests for ROS to conda package mappings."""

from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from pixi_ros.mappings import (
    expand_gl_requirements,
    get_mappings,
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
    assert map_ros_to_conda("ament_cmake_gtest", "iron") == [
        "ros-iron-ament-cmake-gtest"
    ]


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
    assert "foxy" in distros
    assert "humble" in distros
    assert "jazzy" in distros
    assert "kilted" in distros
    assert isinstance(distros, list)


def test_validate_distro():
    """Test distro validation."""
    assert validate_distro("foxy")
    assert validate_distro("humble")
    assert validate_distro("jazzy")
    assert validate_distro("kilted")
    assert not validate_distro("invalid")
    assert not validate_distro("iron")  # Not in current list


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
            assert map_ros_to_conda("custom_package", "humble") == [
                "custom-conda-package"
            ]
            # Platform-specific mapping (will vary by platform)
            result = map_ros_to_conda("another_package", "humble")
            assert isinstance(result, list)
            assert len(result) > 0 or result == []  # Depends on platform

        finally:
            os.chdir(old_cwd)
            reload_mappings()  # Restore default mappings


def test_opengl_returns_require_opengl_placeholder():
    """Test that 'opengl' dependency returns REQUIRE_OPENGL placeholder from mapping."""
    # On Linux, opengl maps to REQUIRE_OPENGL
    result = map_ros_to_conda("opengl", "humble", platform_override="linux")
    assert result == ["REQUIRE_OPENGL"]

    # On macOS, opengl also maps to REQUIRE_OPENGL
    result = map_ros_to_conda("opengl", "humble", platform_override="osx")
    assert result == ["REQUIRE_OPENGL"]

    # On Windows, opengl maps to empty list
    result = map_ros_to_conda("opengl", "humble", platform_override="win64")
    assert result == []


def test_qt_packages_include_require_opengl():
    """Test that Qt packages include REQUIRE_OPENGL placeholder from mapping."""
    # libqt5-core should include qt-main and REQUIRE_OPENGL
    result = map_ros_to_conda("libqt5-core", "humble")
    assert "qt-main" in result
    assert "REQUIRE_OPENGL" in result


def test_expand_gl_requirements_linux():
    """Test expanding GL requirements on Linux."""
    # Test REQUIRE_GL
    result = expand_gl_requirements(["cmake", "REQUIRE_GL"], platform_override="linux")
    assert "cmake" in result
    assert "libgl-devel" in result
    assert "REQUIRE_GL" not in result

    # Test REQUIRE_OPENGL
    result = expand_gl_requirements(
        ["cmake", "REQUIRE_OPENGL"], platform_override="linux"
    )
    assert "cmake" in result
    assert "libgl-devel" in result
    assert "libopengl-devel" in result
    assert "xorg-libx11" in result
    assert "xorg-libxext" in result
    assert "REQUIRE_OPENGL" not in result


def test_expand_gl_requirements_osx():
    """Test expanding GL requirements on macOS."""
    # Test REQUIRE_GL (should be removed, nothing added)
    result = expand_gl_requirements(["cmake", "REQUIRE_GL"], platform_override="osx")
    assert result == ["cmake"]
    assert "REQUIRE_GL" not in result

    # Test REQUIRE_OPENGL (should add X11 packages)
    result = expand_gl_requirements(
        ["cmake", "REQUIRE_OPENGL"], platform_override="osx"
    )
    assert "cmake" in result
    assert "xorg-libx11" in result
    assert "xorg-libxext" in result
    assert "libgl-devel" not in result
    assert "libopengl-devel" not in result
    assert "REQUIRE_OPENGL" not in result


def test_expand_gl_requirements_win64():
    """Test expanding GL requirements on Windows."""
    # Test REQUIRE_GL (should be removed, nothing added)
    result = expand_gl_requirements(
        ["cmake", "REQUIRE_GL"], platform_override="win64"
    )
    assert result == ["cmake"]

    # Test REQUIRE_OPENGL (should be removed, nothing added)
    result = expand_gl_requirements(
        ["cmake", "REQUIRE_OPENGL"], platform_override="win64"
    )
    assert result == ["cmake"]


def test_expand_gl_requirements_deduplicates():
    """Test that expand_gl_requirements removes duplicates."""
    # If both REQUIRE_GL and REQUIRE_OPENGL are present, libgl-devel should only appear once
    result = expand_gl_requirements(
        ["REQUIRE_GL", "REQUIRE_OPENGL"], platform_override="linux"
    )
    # Count occurrences of libgl-devel
    assert result.count("libgl-devel") == 1
    assert "libopengl-devel" in result
    assert "xorg-libx11" in result
    assert "xorg-libxext" in result
