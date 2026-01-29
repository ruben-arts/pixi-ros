"""Tests for package.xml parsing."""

from pathlib import Path

import pytest

from pixi_ros.package_xml import PackageXML

# Path to mock workspace
MOCK_WORKSPACE = Path(__file__).parent / "fixtures" / "mock_workspace" / "src"


def test_parse_python_package():
    """Test parsing a simple Python package."""
    pkg_path = MOCK_WORKSPACE / "my_python_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    assert pkg.name == "my_python_pkg"
    assert pkg.version == "0.1.0"
    assert pkg.format_version == 3
    assert pkg.build_type == "ament_python"
    assert pkg.maintainer == "Jane Developer"
    assert pkg.maintainer_email == "developer@example.com"
    assert pkg.license == "Apache-2.0"

    # Check dependencies
    assert "rclpy" in pkg.depends
    assert "std_msgs" in pkg.depends
    assert "geometry_msgs" in pkg.depends

    # Check test dependencies
    assert "pytest" in pkg.test_depends
    assert "ament_lint_auto" in pkg.test_depends


def test_parse_cpp_package():
    """Test parsing a C++ package with separated build/exec deps."""
    pkg_path = MOCK_WORKSPACE / "my_cpp_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    assert pkg.name == "my_cpp_pkg"
    assert pkg.version == "1.2.3"
    assert pkg.format_version == 3
    assert pkg.build_type == "ament_cmake"
    assert pkg.license == "MIT"

    # Check separated dependencies
    assert "rclcpp" in pkg.build_depends
    assert "std_msgs" in pkg.build_depends
    assert "sensor_msgs" in pkg.build_depends

    assert "rclcpp" in pkg.exec_depends
    assert "std_msgs" in pkg.exec_depends
    assert "sensor_msgs" in pkg.exec_depends

    # Check test dependencies
    assert "ament_cmake_gtest" in pkg.test_depends


def test_parse_mixed_package():
    """Test parsing a complex package with various dependency types."""
    pkg_path = MOCK_WORKSPACE / "my_mixed_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    assert pkg.name == "my_mixed_pkg"
    assert pkg.version == "2.0.0"
    assert pkg.format_version == 3
    assert pkg.license == "BSD"

    # Check buildtool dependencies
    assert "ament_cmake" in pkg.buildtool_depends
    assert "rosidl_default_generators" in pkg.buildtool_depends

    # Check build dependencies
    assert "rclcpp" in pkg.build_depends
    assert "rclpy" in pkg.build_depends

    # Check exec dependencies
    assert "nav_msgs" in pkg.exec_depends
    assert "tf2_ros" in pkg.exec_depends

    # Check build_export dependencies
    assert "geometry_msgs" in pkg.build_export_depends
    assert "std_msgs" in pkg.build_export_depends

    # Check workspace dependencies
    assert "my_cpp_pkg" in pkg.depends
    assert "my_python_pkg" in pkg.exec_depends

    # Check test dependencies
    assert "launch_testing" in pkg.test_depends


def test_parse_legacy_format2_package():
    """Test parsing a legacy format 2 package."""
    pkg_path = MOCK_WORKSPACE / "legacy_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    assert pkg.name == "legacy_pkg"
    assert pkg.version == "0.5.0"
    assert pkg.format_version == 2
    assert pkg.license == "Apache-2.0"

    # Format 2 uses run_depend instead of exec_depend
    assert "roscpp" in pkg.run_depends
    assert "rospy" in pkg.run_depends
    assert "std_msgs" in pkg.run_depends
    assert "actionlib" in pkg.run_depends

    # Check build dependencies
    assert "roscpp" in pkg.build_depends
    assert "rospy" in pkg.build_depends

    # Check test dependencies
    assert "rostest" in pkg.test_depends
    assert "rosunit" in pkg.test_depends


def test_get_all_build_dependencies():
    """Test getting all build dependencies."""
    pkg_path = MOCK_WORKSPACE / "my_mixed_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    build_deps = pkg.get_all_build_dependencies()

    # Should include buildtool, build, and generic depends
    assert "ament_cmake" in build_deps
    assert "rclcpp" in build_deps
    assert "rclpy" in build_deps
    assert "my_cpp_pkg" in build_deps

    # Should be sorted and deduplicated
    assert build_deps == sorted(set(build_deps))


def test_get_all_runtime_dependencies():
    """Test getting all runtime dependencies."""
    pkg_path = MOCK_WORKSPACE / "my_mixed_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    runtime_deps = pkg.get_all_runtime_dependencies()

    # Should include exec and generic depends
    assert "rclcpp" in runtime_deps
    assert "rclpy" in runtime_deps
    assert "nav_msgs" in runtime_deps
    assert "tf2_ros" in runtime_deps
    assert "my_cpp_pkg" in runtime_deps
    assert "my_python_pkg" in runtime_deps

    # Should be sorted and deduplicated
    assert runtime_deps == sorted(set(runtime_deps))


def test_get_all_runtime_dependencies_format2():
    """Test getting runtime dependencies from format 2 package."""
    pkg_path = MOCK_WORKSPACE / "legacy_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    runtime_deps = pkg.get_all_runtime_dependencies()

    # Should include run_depend (format 2)
    assert "roscpp" in runtime_deps
    assert "rospy" in runtime_deps
    assert "actionlib" in runtime_deps


def test_get_all_test_dependencies():
    """Test getting all test dependencies."""
    pkg_path = MOCK_WORKSPACE / "my_python_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    test_deps = pkg.get_all_test_dependencies()

    assert "pytest" in test_deps
    assert "ament_lint_auto" in test_deps
    assert "ament_lint_common" in test_deps


def test_get_all_dependencies():
    """Test getting all dependencies regardless of type."""
    pkg_path = MOCK_WORKSPACE / "my_cpp_pkg" / "package.xml"
    pkg = PackageXML.from_file(pkg_path)

    all_deps = pkg.get_all_dependencies()

    # Should include all types
    assert "ament_cmake" in all_deps  # buildtool
    assert "rclcpp" in all_deps  # build and exec
    assert "ament_cmake_gtest" in all_deps  # test

    # Should be sorted and deduplicated
    assert all_deps == sorted(set(all_deps))


def test_parse_nonexistent_file():
    """Test that parsing a nonexistent file raises FileNotFoundError."""
    with pytest.raises(FileNotFoundError):
        PackageXML.from_file(Path("/nonexistent/package.xml"))


def test_parse_invalid_xml():
    """Test that parsing invalid XML raises ValueError."""
    from tempfile import NamedTemporaryFile

    with NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
        f.write("This is not valid XML")
        f.flush()
        temp_path = Path(f.name)

    try:
        with pytest.raises(ValueError, match="Invalid XML"):
            PackageXML.from_file(temp_path)
    finally:
        temp_path.unlink()


def test_parse_missing_required_fields():
    """Test that parsing XML with missing required fields raises ValueError."""
    from tempfile import NamedTemporaryFile

    with NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
        f.write(
            """<?xml version="1.0"?>
        <package format="3">
            <name>test_pkg</name>
            <!-- Missing version, description, maintainer, license -->
        </package>
        """
        )
        f.flush()
        temp_path = Path(f.name)

    try:
        with pytest.raises(ValueError, match="Missing required"):
            PackageXML.from_file(temp_path)
    finally:
        temp_path.unlink()
