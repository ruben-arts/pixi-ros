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


def test_unix_target_for_linux_and_osx_deps():
    """Test that dependencies on both Linux and macOS go to unix target section."""
    from pixi_ros.init import init_workspace

    with TemporaryDirectory() as tmpdir:
        workspace_path = Path(tmpdir)
        src_dir = workspace_path / "src"
        src_dir.mkdir()

        # Create package.xml with opengl dependency
        # OpenGL requires X11 packages on both linux and osx, but different GL packages on linux only
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

        # Initialize with Linux and macOS (no Windows)
        init_workspace("humble", workspace_path, platforms=["linux-64", "osx-arm64"])

        # Check pixi.toml was created
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        # Parse and check target sections
        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        # Unix target section should exist for shared dependencies
        if "target" in config and "unix" in config["target"]:
            assert "dependencies" in config["target"]["unix"]
            unix_deps = config["target"]["unix"]["dependencies"]
            # X11 packages should be in unix (shared between linux and osx)
            assert any("x11" in str(dep).lower() or "xorg" in str(dep).lower() for dep in unix_deps.keys())

        # Linux-specific section should have GL packages not in unix
        if "target" in config and "linux" in config["target"]:
            assert "dependencies" in config["target"]["linux"]
            linux_deps = config["target"]["linux"]["dependencies"]
            # GL packages specific to Linux
            assert any("libgl-devel" in str(dep).lower() or "libopengl-devel" in str(dep).lower() for dep in linux_deps.keys())


def test_tasks_have_descriptions():
    """Test that generated tasks include descriptions."""
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

        # Initialize workspace
        init_workspace("humble", workspace_path, platforms=["linux-64"])

        # Check pixi.toml was created
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        # Parse and check tasks
        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        assert "tasks" in config
        tasks = config["tasks"]

        # Check that expected tasks exist and have descriptions
        expected_tasks = {
            "build": {
                "cmd": "colcon build",
                "description": "Build the ROS workspace",
            },
            "test": {
                "cmd": "colcon test",
                "description": "Run tests for the workspace",
            },
            "clean": {
                "cmd": "rm -rf build install log",
                "description": "Clean build artifacts (build, install, log directories)",
            },
        }

        for task_name, expected_config in expected_tasks.items():
            assert task_name in tasks, f"Task '{task_name}' not found in pixi.toml"
            task = tasks[task_name]

            # Task should be a dict/table with cmd and description
            assert isinstance(task, dict), f"Task '{task_name}' should be a dictionary"
            assert "cmd" in task, f"Task '{task_name}' missing 'cmd' field"
            assert "description" in task, f"Task '{task_name}' missing 'description' field"

            # Verify the content matches
            assert task["cmd"] == expected_config["cmd"], f"Task '{task_name}' has wrong command"
            assert task["description"] == expected_config["description"], f"Task '{task_name}' has wrong description"


def test_platforms_extended_not_overridden():
    """Test that running init multiple times extends the platforms list instead of overriding it."""
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

        # Check pixi.toml was created with linux-64
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        assert "workspace" in config
        assert "platforms" in config["workspace"]
        assert config["workspace"]["platforms"] == ["linux-64"]

        # Initialize again with additional platforms
        init_workspace("humble", workspace_path, platforms=["osx-arm64", "win-64"])

        # Read updated config
        with open(toml_path) as f:
            config = tomlkit.load(f)

        # Verify all platforms are present
        platforms = config["workspace"]["platforms"]
        assert "linux-64" in platforms, "Original platform should still be present"
        assert "osx-arm64" in platforms, "New platform osx-arm64 should be added"
        assert "win-64" in platforms, "New platform win-64 should be added"
        assert len(platforms) == 3, "Should have exactly 3 platforms"

        # Verify no duplicates if we run init again with overlapping platforms
        init_workspace("humble", workspace_path, platforms=["linux-64", "osx-64"])

        with open(toml_path) as f:
            config = tomlkit.load(f)

        platforms = config["workspace"]["platforms"]
        # linux-64 should not be duplicated
        assert platforms.count("linux-64") == 1, "linux-64 should not be duplicated"
        # osx-64 should be added (new)
        assert "osx-64" in platforms, "New platform osx-64 should be added"
        # All previous platforms should still be there
        assert "osx-arm64" in platforms
        assert "win-64" in platforms


def test_version_constraints_from_package_xml():
    """Test that version constraints from package.xml are applied to pixi.toml dependencies."""
    from pixi_ros.init import init_workspace

    with TemporaryDirectory() as tmpdir:
        workspace_path = Path(tmpdir)
        src_dir = workspace_path / "src"
        src_dir.mkdir()

        # Create a package.xml with version-constrained dependencies
        pkg_xml = src_dir / "package.xml"
        pkg_xml.write_text("""<?xml version="1.0"?>
<package format="2">
    <name>test_pkg</name>
    <version>0.0.1</version>
    <description>Test</description>
    <maintainer email="test@test.com">Test</maintainer>
    <license>MIT</license>
    <depend version_gte="3.12.4">cmake</depend>
    <build_depend version_gte="3.3.0" version_lt="4.0.0">eigen</build_depend>
    <exec_depend version_eq="1.2.3">boost</exec_depend>
</package>
""")

        # Initialize workspace
        init_workspace("humble", workspace_path, platforms=["linux-64"])

        # Check pixi.toml was created
        toml_path = workspace_path / "pixi.toml"
        assert toml_path.exists()

        # Parse and check dependencies
        import tomlkit
        with open(toml_path) as f:
            config = tomlkit.load(f)

        assert "dependencies" in config
        dependencies = config["dependencies"]

        # Check that cmake has the version constraint
        # Note: cmake might already have a constraint from CMakeLists.txt detection
        # so we just verify it has some constraint
        if "cmake" in dependencies:
            cmake_version = dependencies["cmake"]
            assert cmake_version != "*", "cmake should have a version constraint"
            assert ">=" in cmake_version or "<" in cmake_version, "cmake should have a version constraint operator"

        # Check that eigen has the version constraint (>=3.3.0,<4.0.0)
        if "eigen" in dependencies:
            eigen_version = dependencies["eigen"]
            assert eigen_version != "*", "eigen should have a version constraint"
            assert ">=3.3.0" in eigen_version or ">=" in eigen_version, "eigen should have >= constraint"

        # Check that boost has the exact version constraint (==1.2.3)
        if "boost" in dependencies:
            boost_version = dependencies["boost"]
            assert boost_version != "*", "boost should have a version constraint"
            assert "==" in boost_version or "1.2.3" in str(boost_version), "boost should have == constraint"
