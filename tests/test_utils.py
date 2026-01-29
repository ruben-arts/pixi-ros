"""Tests for utility functions."""

import pytest

from pixi_ros.utils import is_valid_ros_package_name


@pytest.mark.parametrize(
    "name,expected",
    [
        ("my_package", True),
        ("my_package_2", True),
        ("package", True),
        ("a", True),
        ("_package", False),  # Must start with letter
        ("2package", False),  # Must start with letter
        ("MyPackage", False),  # Must be lowercase
        ("my-package", False),  # No hyphens allowed
        ("my__package", False),  # No consecutive underscores
        ("my package", False),  # No spaces
        ("", False),  # Empty string
    ],
)
def test_is_valid_ros_package_name(name, expected):
    """Test ROS package name validation."""
    assert is_valid_ros_package_name(name) == expected
