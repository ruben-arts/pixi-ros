"""Tests for workspace discovery."""

from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from pixi_ros.workspace import (
    discover_packages,
    find_package_xml,
    find_workspace_root,
    is_workspace_package,
)

# Path to mock workspace
MOCK_WORKSPACE = Path(__file__).parent / "fixtures" / "mock_workspace"


def test_find_workspace_root_from_workspace():
    """Test finding workspace root from workspace directory."""
    root = find_workspace_root(MOCK_WORKSPACE)
    assert root == MOCK_WORKSPACE


def test_find_workspace_root_from_package():
    """Test finding workspace root from inside a package."""
    package_dir = MOCK_WORKSPACE / "src" / "my_python_pkg"
    root = find_workspace_root(package_dir)
    assert root == MOCK_WORKSPACE


def test_find_workspace_root_from_src():
    """Test finding workspace root from src directory."""
    src_dir = MOCK_WORKSPACE / "src"
    root = find_workspace_root(src_dir)
    assert root == MOCK_WORKSPACE


def test_find_package_xml_from_package():
    """Test finding package.xml from package directory."""
    package_dir = MOCK_WORKSPACE / "src" / "my_python_pkg"
    pkg_xml = find_package_xml(package_dir)
    assert pkg_xml == package_dir / "package.xml"


def test_find_package_xml_from_subdir():
    """Test finding package.xml from subdirectory of package."""
    # Create a temporary subdirectory
    package_dir = MOCK_WORKSPACE / "src" / "my_python_pkg"
    pkg_xml = find_package_xml(package_dir)
    assert pkg_xml is not None
    assert pkg_xml.name == "package.xml"


def test_find_package_xml_not_found():
    """Test that None is returned when package.xml is not found."""
    with TemporaryDirectory() as tmpdir:
        tmppath = Path(tmpdir)
        result = find_package_xml(tmppath)
        assert result is None


def test_discover_packages():
    """Test discovering all packages in workspace."""
    packages = discover_packages(MOCK_WORKSPACE)

    # Should find all 4 packages
    assert len(packages) == 4

    package_names = {pkg.name for pkg in packages}
    assert "my_python_pkg" in package_names
    assert "my_cpp_pkg" in package_names
    assert "my_mixed_pkg" in package_names
    assert "legacy_pkg" in package_names


def test_discover_packages_details():
    """Test that discovered packages have correct details."""
    packages = discover_packages(MOCK_WORKSPACE)

    # Find specific package
    python_pkg = next(p for p in packages if p.name == "my_python_pkg")
    assert python_pkg.version == "0.1.0"
    assert "rclpy" in python_pkg.depends

    cpp_pkg = next(p for p in packages if p.name == "my_cpp_pkg")
    assert cpp_pkg.version == "1.2.3"
    assert "rclcpp" in cpp_pkg.build_depends


def test_discover_packages_invalid_workspace():
    """Test that discovering packages in invalid workspace raises error."""
    with pytest.raises(ValueError, match="does not exist"):
        discover_packages(Path("/nonexistent"))

    with TemporaryDirectory() as tmpdir:
        # Create a file, not a directory
        file_path = Path(tmpdir) / "not_a_dir"
        file_path.touch()

        with pytest.raises(ValueError, match="not a directory"):
            discover_packages(file_path)


def test_is_workspace_package():
    """Test checking if a package is in the workspace."""
    packages = discover_packages(MOCK_WORKSPACE)

    assert is_workspace_package("my_python_pkg", packages)
    assert is_workspace_package("my_cpp_pkg", packages)
    assert is_workspace_package("my_mixed_pkg", packages)
    assert not is_workspace_package("std_msgs", packages)
    assert not is_workspace_package("rclcpp", packages)
    assert not is_workspace_package("nonexistent_pkg", packages)


def test_find_workspace_root_single_package():
    """Test finding workspace root for single-package workspace."""
    with TemporaryDirectory() as tmpdir:
        tmppath = Path(tmpdir).resolve()

        # Create a simple package.xml
        pkg_xml = tmppath / "package.xml"
        pkg_xml.write_text(
            """<?xml version="1.0"?>
        <package format="3">
            <name>test_pkg</name>
            <version>0.1.0</version>
            <description>Test</description>
            <maintainer email="test@test.com">Test</maintainer>
            <license>MIT</license>
        </package>
        """
        )

        # Should find the package
        result = find_package_xml(tmppath)
        assert result is not None
        assert result.resolve() == pkg_xml.resolve()


def test_discover_packages_empty_workspace():
    """Test discovering packages in empty workspace."""
    with TemporaryDirectory() as tmpdir:
        tmppath = Path(tmpdir)
        (tmppath / "src").mkdir()

        packages = discover_packages(tmppath)
        assert len(packages) == 0


def test_discover_packages_respects_gitignore():
    """Test that package discovery respects .gitignore patterns."""
    with TemporaryDirectory() as tmpdir:
        tmppath = Path(tmpdir)

        # Create a gitignore that excludes ignored_pkg
        gitignore = tmppath / ".gitignore"
        gitignore.write_text("ignored_pkg/\n")

        # Create two packages - one that should be ignored, one that shouldn't
        for pkg_name in ["valid_pkg", "ignored_pkg"]:
            pkg_dir = tmppath / pkg_name
            pkg_dir.mkdir()
            pkg_xml = pkg_dir / "package.xml"
            pkg_xml.write_text(
                f"""<?xml version="1.0"?>
            <package format="3">
                <name>{pkg_name}</name>
                <version>0.1.0</version>
                <description>Test</description>
                <maintainer email="test@test.com">Test</maintainer>
                <license>MIT</license>
            </package>
            """
            )

        packages = discover_packages(tmppath)

        # Should only find valid_pkg, not ignored_pkg
        assert len(packages) == 1
        assert packages[0].name == "valid_pkg"
