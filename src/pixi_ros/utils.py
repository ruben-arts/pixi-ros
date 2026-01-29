"""Shared utility functions for pixi-ros."""

import re
from pathlib import Path

from rattler import Version, VersionSpec


def detect_cmake_version_requirement(package_path: Path) -> str | None:
    """
    Detect cmake version requirement from CMakeLists.txt.

    Args:
        package_path: Path to the package directory

    Returns:
        Version constraint for cmake, or None if not applicable
    """
    cmake_file = package_path / "CMakeLists.txt"
    if not cmake_file.exists():
        return None

    try:
        content = cmake_file.read_text()
        # Look for cmake_minimum_required(VERSION x.y)
        match = re.search(
            r"cmake_minimum_required\s*\(\s*VERSION\s+([\d.]+)",
            content,
            re.IGNORECASE,
        )
        if match:
            version_str = match.group(1)
            # Parse the detected version
            detected_version = Version(version_str)

            # Check if version is less than 3.10
            threshold_spec = VersionSpec("<3.10")
            if threshold_spec.matches(detected_version):
                # CMake versions < 3.10 require cmake <4
                return "<4"

    except Exception:
        # If we can't read or parse the file, skip it
        pass

    return None


def is_valid_ros_package_name(name: str) -> bool:
    """
    Check if a package name follows ROS naming conventions.

    ROS package names should:
    - Only contain lowercase letters, numbers, and underscores
    - Start with a letter
    - Not contain consecutive underscores

    Args:
        name: Package name to validate

    Returns:
        True if name is valid, False otherwise
    """
    if not name:
        return False

    # Must start with a letter
    if not name[0].isalpha():
        return False

    # Check each character
    for char in name:
        if not (char.islower() or char.isdigit() or char == "_"):
            return False

    # No consecutive underscores
    if "__" in name:
        return False

    return True
