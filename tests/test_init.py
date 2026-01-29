"""Tests for initialization and dependency detection logic."""

from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from pixi_ros.utils import detect_cmake_version_requirement


def test_cmake_version_below_3_10_requires_constraint():
    """Test that CMake versions below 3.10 get the <4 constraint."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        cmake_file = pkg_path / "CMakeLists.txt"

        # Test various versions below 3.10
        test_versions = ["3.8", "3.5", "3.9", "3.0", "3.9.1"]

        for version in test_versions:
            cmake_file.write_text(f"cmake_minimum_required(VERSION {version})\n")
            constraint = detect_cmake_version_requirement(pkg_path)
            assert constraint == "<4", f"Version {version} should require <4 constraint"


def test_cmake_version_3_10_and_above_no_constraint():
    """Test that CMake versions 3.10 and above don't get a constraint."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        cmake_file = pkg_path / "CMakeLists.txt"

        # Test versions 3.10 and above
        test_versions = ["3.10", "3.15", "3.20", "3.25", "3.10.0", "3.29.2"]

        for version in test_versions:
            cmake_file.write_text(f"cmake_minimum_required(VERSION {version})\n")
            constraint = detect_cmake_version_requirement(pkg_path)
            assert constraint is None, f"Version {version} should not require constraint"


def test_no_cmake_file_returns_none():
    """Test that missing CMakeLists.txt returns None."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        constraint = detect_cmake_version_requirement(pkg_path)
        assert constraint is None


def test_cmake_version_case_insensitive():
    """Test that cmake_minimum_required matching is case-insensitive."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        cmake_file = pkg_path / "CMakeLists.txt"

        # Test different case variations
        test_cases = [
            "cmake_minimum_required(VERSION 3.8)",
            "CMAKE_MINIMUM_REQUIRED(VERSION 3.8)",
            "CmAkE_MiNiMuM_rEqUiReD(VERSION 3.8)",
        ]

        for content in test_cases:
            cmake_file.write_text(content)
            constraint = detect_cmake_version_requirement(pkg_path)
            assert constraint == "<4"


def test_cmake_version_with_whitespace():
    """Test that whitespace variations are handled correctly."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        cmake_file = pkg_path / "CMakeLists.txt"

        # Test different whitespace patterns
        test_cases = [
            "cmake_minimum_required( VERSION 3.8 )",
            "cmake_minimum_required(  VERSION  3.8  )",
            "cmake_minimum_required(\tVERSION\t3.8\t)",
        ]

        for content in test_cases:
            cmake_file.write_text(content)
            constraint = detect_cmake_version_requirement(pkg_path)
            assert constraint == "<4"


def test_cmake_version_in_larger_file():
    """Test detection when cmake_minimum_required is in a larger file."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        cmake_file = pkg_path / "CMakeLists.txt"

        content = """
# This is a comment
project(my_package)

cmake_minimum_required(VERSION 3.8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
        """

        cmake_file.write_text(content)
        constraint = detect_cmake_version_requirement(pkg_path)
        assert constraint == "<4"


def test_malformed_cmake_file():
    """Test that malformed CMakeLists.txt doesn't crash."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        cmake_file = pkg_path / "CMakeLists.txt"

        # Test various malformed content
        test_cases = [
            "cmake_minimum_required(VERSION invalid)",
            "cmake_minimum_required()",
            "some random content",
            "",
        ]

        for content in test_cases:
            cmake_file.write_text(content)
            constraint = detect_cmake_version_requirement(pkg_path)
            # Should return None for unparseable content
            assert constraint is None


def test_cmake_version_4_and_above():
    """Test that CMake 4.x and above don't get a constraint."""
    with TemporaryDirectory() as tmpdir:
        pkg_path = Path(tmpdir)
        cmake_file = pkg_path / "CMakeLists.txt"

        # Test CMake 4.x versions
        test_versions = ["4.0", "4.5", "5.0"]

        for version in test_versions:
            cmake_file.write_text(f"cmake_minimum_required(VERSION {version})\n")
            constraint = detect_cmake_version_requirement(pkg_path)
            assert constraint is None, f"Version {version} should not require constraint"


def test_platforms_added_to_workspace_single_platform():
    """Test that single platform is added to workspace section."""
    from pixi_ros.init import init_workspace

    with TemporaryDirectory() as tmpdir:
        workspace_path = Path(tmpdir)
        src_dir = workspace_path / "src"
        src_dir.mkdir()

        # Create a simple package.xml
        pkg_xml = src_dir / "package.xml"
        pkg_xml.write_text("""<?xml version="1.0"?>
<package format="2">
    <name>test_pkg</name>
    <version>0.0.1</version>
    <description>Test</description>
    <maintainer email="test@test.com">Test</maintainer>
    <license>MIT</license>
</package>
""")

        # Initialize with single platform
        init_workspace("humble", workspace_path, platforms=["linux-64"])

        # Check pixi.toml was created
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        # Parse and check platforms
        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        assert "workspace" in config
        assert "platforms" in config["workspace"]
        assert config["workspace"]["platforms"] == ["linux-64"]


def test_platforms_added_to_workspace_multiple_platforms():
    """Test that multiple platforms are added correctly to workspace section."""
    from pixi_ros.init import init_workspace

    with TemporaryDirectory() as tmpdir:
        workspace_path = Path(tmpdir)
        src_dir = workspace_path / "src"
        src_dir.mkdir()

        # Create a simple package.xml
        pkg_xml = src_dir / "package.xml"
        pkg_xml.write_text("""<?xml version="1.0"?>
<package format="2">
    <name>test_pkg</name>
    <version>0.0.1</version>
    <description>Test</description>
    <maintainer email="test@test.com">Test</maintainer>
    <license>MIT</license>
</package>
""")

        # Initialize with multiple platforms
        init_workspace("humble", workspace_path, platforms=["linux-64", "osx-64", "osx-arm64", "win-64"])

        # Check pixi.toml was created
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        # Parse and check platforms
        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        assert "workspace" in config
        assert "platforms" in config["workspace"]
        # All platforms should be in the list
        platforms = config["workspace"]["platforms"]
        assert platforms == ["linux-64", "osx-64", "osx-arm64", "win-64"]


def test_target_sections_created_for_platform_specific_deps():
    """Test that target.{platform}.dependencies sections are created."""
    from pixi_ros.init import init_workspace

    with TemporaryDirectory() as tmpdir:
        workspace_path = Path(tmpdir)
        src_dir = workspace_path / "src"
        src_dir.mkdir()

        # Create package.xml with opengl dependency (platform-specific)
        pkg_xml = src_dir / "package.xml"
        pkg_xml.write_text("""<?xml version="1.0"?>
<package format="2">
    <name>test_pkg</name>
    <version>0.0.1</version>
    <description>Test</description>
    <maintainer email="test@test.com">Test</maintainer>
    <license>MIT</license>
    <depend>opengl</depend>
</package>
""")

        # Initialize with multiple platforms
        init_workspace("humble", workspace_path, platforms=["linux-64", "osx-arm64"])

        # Check pixi.toml was created
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        # Parse and check target sections
        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        # Target sections should exist for platform-specific deps
        assert "target" in config

        # Linux should have GL deps
        if "linux" in config["target"]:
            assert "dependencies" in config["target"]["linux"]
            linux_deps = config["target"]["linux"]["dependencies"]
            # Check for GL-related packages
            assert any("gl" in str(dep).lower() for dep in linux_deps.keys())

        # OSX should have X11 deps
        if "osx" in config["target"]:
            assert "dependencies" in config["target"]["osx"]
            osx_deps = config["target"]["osx"]["dependencies"]
            # Check for X11-related packages
            assert any("x11" in str(dep).lower() or "xorg" in str(dep).lower() for dep in osx_deps.keys())


def test_common_dependencies_in_main_section():
    """Test that common dependencies (available on all platforms) go to main dependencies section."""
    from pixi_ros.init import init_workspace

    with TemporaryDirectory() as tmpdir:
        workspace_path = Path(tmpdir)
        src_dir = workspace_path / "src"
        src_dir.mkdir()

        # Create package.xml with common dependency
        pkg_xml = src_dir / "package.xml"
        pkg_xml.write_text("""<?xml version="1.0"?>
<package format="2">
    <name>test_pkg</name>
    <version>0.0.1</version>
    <description>Test</description>
    <maintainer email="test@test.com">Test</maintainer>
    <license>MIT</license>
    <depend>rclcpp</depend>
</package>
""")

        # Initialize with multiple platforms
        init_workspace("humble", workspace_path, platforms=["linux-64", "osx-arm64"])

        # Check pixi.toml was created
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        # Parse and check dependencies
        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        # Common dependency should be in main dependencies
        assert "dependencies" in config
        dependencies = config["dependencies"]

        # Check for rclcpp (common across platforms)
        assert any("rclcpp" in str(dep) for dep in dependencies.keys())
