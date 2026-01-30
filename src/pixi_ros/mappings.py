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

    Note: The returned list may contain special placeholders like REQUIRE_GL
    or REQUIRE_OPENGL (from the mapping files). These should be expanded
    using expand_gl_requirements().

    Args:
        ros_package: The ROS package name (e.g., "rclcpp", "udev", "opengl")
        distro: The ROS distribution (e.g., "humble", "iron", "jazzy")
        platform_override: Override platform detection (for testing)

    Returns:
        List of conda package names, which may include placeholder strings
        like REQUIRE_GL or REQUIRE_OPENGL that need expansion
        (e.g., ["ros-humble-rclcpp"], ["libusb", "libudev"], or ["REQUIRE_OPENGL"])

    Examples:
        >>> map_ros_to_conda("udev", "humble")  # doctest: +SKIP
        ['libusb', 'libudev']  # on linux
        >>> map_ros_to_conda("uncrustify", "humble")
        ['uncrustify']
        >>> map_ros_to_conda("opengl", "humble")  # doctest: +SKIP
        ['REQUIRE_OPENGL']  # placeholder from mapping file
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


def expand_gl_requirements(
    conda_packages: list[str], platform_override: str | None = None
) -> list[str]:
    """
    Process special GL requirements in a list of conda packages.

    Replaces REQUIRE_GL and REQUIRE_OPENGL placeholders with actual
    platform-specific conda packages.

    This is a duplication of the code in:
    https://github.com/RoboStack/vinca/blob/7d3a05e01d6898201a66ba2cf6ea771250671f58/vinca/main.py#L562

    Args:
        conda_packages: List of conda package names (may contain REQUIRE_(OPEN)GL)
        platform_override: Override platform detection (for testing)

    Returns:
        List of conda packages with GL requirements expanded

    Examples:
        >>> expand_gl_requirements(["cmake", "REQUIRE_GL"], platform_override="linux")
        ['cmake', 'libgl-devel']
        >>> expand_gl_requirements(["REQUIRE_GL"], platform_override="osx")
        []
    """
    current_platform = platform_override or _detect_platform()
    result = []
    additional_packages = []

    for pkg in conda_packages:
        if pkg == "REQUIRE_GL":
            # Replace REQUIRE_GL with platform-specific packages
            if current_platform == "linux":
                additional_packages.append("libgl-devel")
            # On other platforms, just remove it (add nothing)
        elif pkg == "REQUIRE_OPENGL":
            # Replace REQUIRE_OPENGL with platform-specific packages
            if current_platform == "linux":
                # TODO: this should only go into the host dependencies
                additional_packages.extend(["libgl-devel", "libopengl-devel"])
            if current_platform in ["linux", "osx"]:
                # TODO: force this into the run dependencies
                additional_packages.extend(["xorg-libx11", "xorg-libxext"])
            # On windows, just remove it (add nothing)
        else:
            # Regular package, keep it
            result.append(pkg)

    # Add the additional packages and deduplicate
    result.extend(additional_packages)

    # Remove duplicates while preserving order
    seen = set()
    deduplicated = []
    for pkg in result:
        if pkg not in seen:
            seen.add(pkg)
            deduplicated.append(pkg)

    return deduplicated


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


def get_platforms() -> list[str]:
    """
    Get list of supported pixi platforms based on mapping files.

    Extracts platform names from the mapping data and converts them to
    standard pixi platform names.

    Mapping files use: linux, osx, win64
    Pixi uses: linux-64, osx-64, osx-arm64, win-64

    Returns:
        List of pixi platform names
    """
    mappings = get_mappings()
    mapping_platforms = set()

    # Iterate through mappings to find all platform keys
    for package_mappings in mappings.values():
        for channel_mapping in package_mappings.values():
            if isinstance(channel_mapping, dict):
                # This is a platform-specific mapping
                mapping_platforms.update(channel_mapping.keys())

    # Convert mapping platforms to pixi platforms
    pixi_platforms = []
    if "linux" in mapping_platforms:
        pixi_platforms.append("linux-64")
    if "osx" in mapping_platforms:
        pixi_platforms.extend(["osx-64", "osx-arm64"])
    if "win64" in mapping_platforms or "win" in mapping_platforms:
        pixi_platforms.append("win-64")

    return (
        pixi_platforms
        if pixi_platforms
        else ["linux-64", "osx-64", "osx-arm64", "win-64"]
    )


def get_ros_distros() -> list[str]:
    """
    Get list of supported ROS distributions.

    Returns:
        List of ROS distro names
    """
    return ["foxy", "humble", "jazzy", "kilted"]


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
