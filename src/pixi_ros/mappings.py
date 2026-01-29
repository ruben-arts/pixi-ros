"""Mapping between ROS package names and conda package names."""

from functools import lru_cache
from pathlib import Path

import yaml
from rattler import Platform


def _detect_platform() -> str:
    """
    Detect the current platform using rattler.

    Returns:
        Platform name: 'linux', 'osx', 'win64', etc.
    """
    current_platform = Platform.current()
    platform_str = str(current_platform)

    # Map rattler platform names to ROS platform names
    if platform_str.startswith("osx"):
        return "osx"
    elif platform_str.startswith("win"):
        return "win64"
    elif platform_str.startswith("linux"):
        return "linux"
    else:
        # Default to linux for unknown platforms
        return "linux"


@lru_cache(maxsize=1)
def _load_channel_mappings() -> dict[str, dict[str, list[str] | dict[str, list[str]]]]:
    """
    Load ROS to conda package mappings from channel.yaml files.

    The new format is:
    ```yaml
    package_name:
      channel_name:
        - simple_mapping
      other_channel:
        linux: [pkg1, pkg2]
        osx: [pkg3]
        win64: []
    ```

    Searches for *.yaml files in the following order:
    1. Current workspace directory (./pixi-ros/*.yaml)
    2. User config directory (~/.pixi-ros/*.yaml)
    3. Built-in defaults (packaged with pixi-ros)

    Returns:
        Dictionary mapping ROS package names to channel/platform mappings
    """
    mappings: dict[str, dict[str, list[str] | dict[str, list[str]]]] = {}

    # Search locations in priority order
    search_dirs = [
        Path.cwd() / "pixi-ros",
        Path.home() / ".pixi-ros",
        Path(__file__).parent / "data",
    ]

    for search_dir in search_dirs:
        if not search_dir.exists():
            continue

        # Load all .yaml files in the directory
        for yaml_file in search_dir.glob("*.yaml"):
            try:
                with open(yaml_file) as f:
                    data = yaml.safe_load(f)
                    if data:
                        # Merge mappings, with earlier directories taking priority
                        for package, channels in data.items():
                            if package not in mappings:
                                mappings[package] = channels
            except Exception:
                # Skip files that fail to parse
                continue

    return mappings


def get_mappings() -> dict[str, dict[str, list[str] | dict[str, list[str]]]]:
    """
    Get the current ROS to conda package mappings.

    Returns:
        Dictionary mapping ROS package names to channel/platform mappings
    """
    return _load_channel_mappings()


def reload_mappings():
    """
    Clear the cached mappings and force reload.

    Useful for testing or when mapping files have been updated.
    """
    _load_channel_mappings.cache_clear()


def map_ros_to_conda(
    ros_package: str, distro: str = "humble", platform_override: str | None = None
) -> list[str]:
    """
    Map a ROS package name to its conda package names.

    Args:
        ros_package: The ROS package name (e.g., "rclcpp", "udev")
        distro: The ROS distribution (e.g., "humble", "iron", "jazzy")
        platform_override: Override platform detection (for testing)

    Returns:
        List of conda package names
        (e.g., ["ros-humble-rclcpp"] or ["libusb", "libudev"])

    Examples:
        >>> map_ros_to_conda("udev", "humble")  # doctest: +SKIP
        ['libusb', 'libudev']  # on linux
        >>> map_ros_to_conda("uncrustify", "humble")
        ['uncrustify']
    """
    mappings = get_mappings()
    current_platform = platform_override or _detect_platform()

    # Check if we have a mapping for this package
    if ros_package in mappings:
        channels = mappings[ros_package]

        # Get the first channel's mapping (usually "pixi")
        if channels:
            # Take the first channel
            channel_mapping = next(iter(channels.values()))

            # Check if it's platform-specific or a simple list
            if isinstance(channel_mapping, dict):
                # Platform-specific mapping
                packages = channel_mapping.get(current_platform, [])
                return packages if packages else []
            elif isinstance(channel_mapping, list):
                # Simple list mapping
                return channel_mapping

    # Default mapping: convert underscores to dashes and prepend ros-distro
    # This follows the robostack convention
    conda_name = ros_package.replace("_", "-")
    return [f"ros-{distro}-{conda_name}"]


def is_system_package(package_name: str) -> bool:
    """
    Check if a package is a system/tool package rather than a ROS package.

    System packages are those that have explicit mappings and don't use
    the ros-distro prefix.

    Args:
        package_name: The package name to check

    Returns:
        True if it's a system package, False otherwise

    Examples:
        >>> is_system_package("cmake")
        True
        >>> is_system_package("rclcpp")
        False
    """
    mappings = get_mappings()

    # If package is in mappings, it's a system package
    # (ROS packages without mappings will get ros-{distro}- prefix in fallback)
    if package_name in mappings:
        return True

    # Known system packages not in mapping files
    system_packages = {
        "cmake",
        "git",
        "python",
        "python3",
    }
    return package_name in system_packages


def get_ros_distros() -> list[str]:
    """
    Get list of supported ROS distributions.

    Returns:
        List of ROS distro names
    """
    return ["humble", "iron", "jazzy", "rolling"]


def validate_distro(distro: str) -> bool:
    """
    Check if a ROS distro is supported.

    Args:
        distro: The ROS distribution name

    Returns:
        True if supported, False otherwise
    """
    return distro in get_ros_distros()


def get_mapping_files_dir() -> Path:
    """
    Get the directory containing the currently active mapping files.

    Returns:
        Path to directory containing channel.yaml files
    """
    search_dirs = [
        Path.cwd() / "pixi-ros",
        Path.home() / ".pixi-ros",
    ]

    for search_dir in search_dirs:
        if search_dir.exists() and any(search_dir.glob("*.yaml")):
            return search_dir

    # Return built-in default path
    return Path(__file__).parent / "data"


# Deprecated: kept for backward compatibility
def get_robostack_yaml_path() -> Path | None:
    """
    Get the path to mapping files directory.

    Deprecated: Use get_mapping_files_dir() instead.

    Returns:
        Path to first .yaml file found, or None
    """
    mapping_dir = get_mapping_files_dir()
    yaml_files = list(mapping_dir.glob("*.yaml"))
    return yaml_files[0] if yaml_files else None
