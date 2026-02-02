"""ROS package validation logic."""

import asyncio
from dataclasses import dataclass
from enum import Enum

from rattler import Channel, Gateway, Platform
from rosdistro import get_cached_distribution, get_index, get_index_url


class PackageSource(Enum):
    """Source of a package."""

    WORKSPACE = "workspace"
    MAPPING = "mapping"
    ROS_DISTRO = "ros_distro"
    CONDA_FORGE = "conda_forge"
    NOT_FOUND = "not_found"


@dataclass
class PackageValidationResult:
    """Result of package validation."""

    package_name: str
    source: PackageSource
    conda_packages: list[str]
    error: str | None = None


class RosDistroValidator:
    """Validator for ROS packages using rosdistro."""

    def __init__(self, distro_name: str):
        """
        Initialize validator with ROS distribution.

        Args:
            distro_name: ROS distribution name (e.g., "humble", "jazzy")
        """
        self.distro_name = distro_name
        self._distro = None
        self._init_error = None
        self._conda_forge_cache = {}

        try:
            index = get_index(get_index_url())
            self._distro = get_cached_distribution(index, distro_name)
        except Exception as e:
            self._init_error = str(e)

    def has_package(self, package_name: str) -> bool:
        """
        Check if package exists in ROS distribution.

        Args:
            package_name: ROS package name

        Returns:
            True if package exists in distribution
        """
        if self._distro is None:
            return False
        return package_name in self._distro.release_packages

    def check_package_availability(
        self, package_name: str, platform: str, channel_url: str
    ) -> bool:
        """
        Check if package is available in the specified channel.

        Args:
            package_name: Conda package name
            platform: Platform string (e.g., "linux-64", "osx-arm64")
            channel_url: Channel URL to check (e.g., "https://prefix.dev/conda-forge")

        Returns:
            True if package is available in the channel
        """
        # Check cache first
        cache_key = (package_name, platform, channel_url)
        if cache_key in self._conda_forge_cache:
            return self._conda_forge_cache[cache_key]

        try:
            gateway = Gateway()
            channel = Channel(channel_url)
            platform_obj = Platform(platform)
            noarch_obj = Platform("noarch")

            # Query with 10 second timeout, check both platform and noarch
            repo_data = asyncio.wait_for(
                gateway.query(
                    [channel],
                    [platform_obj, noarch_obj],
                    specs=[package_name],
                    recursive=False,
                ),
                timeout=10.0,
            )

            # Check if any records match
            results = asyncio.run(repo_data)
            for channel_records in results:
                for record in channel_records:
                    if record.name.normalized == package_name.lower():
                        self._conda_forge_cache[cache_key] = True
                        return True

            self._conda_forge_cache[cache_key] = False
            return False
        except (asyncio.TimeoutError, Exception):
            # On error or timeout, assume not available
            self._conda_forge_cache[cache_key] = False
            return False

    def check_conda_forge_availability(self, package_name: str, platform: str) -> bool:
        """
        Check if package is available on conda-forge.

        Args:
            package_name: Conda package name
            platform: Platform string (e.g., "linux-64", "osx-arm64")

        Returns:
            True if package is available on conda-forge
        """
        return self.check_package_availability(
            package_name, platform, "https://prefix.dev/conda-forge"
        )

    def validate_package(
        self,
        package_name: str,
        workspace_packages: set[str],
        mappings: dict[str, dict[str, list[str] | dict[str, list[str]]]],
        platform: str = "linux-64",
    ) -> PackageValidationResult:
        """
        Validate a ROS package and determine its source.

        Process:
        1. Determine source (workspace/mapping/ros_distro/conda_forge/not_found)
        2. Validate that packages actually exist in their expected channels

        Args:
            package_name: ROS package name
            workspace_packages: Set of package names in the workspace
            mappings: Package mappings from mapping files
            platform: Target platform (default: "linux-64")

        Returns:
            PackageValidationResult with source and conda package names
        """
        # Step 1: Determine source without validation
        source = None
        conda_packages = []

        # 1. Check if it's a workspace package
        if package_name in workspace_packages:
            return PackageValidationResult(
                package_name=package_name,
                source=PackageSource.WORKSPACE,
                conda_packages=[],
            )

        # 2. Check if it's in the mapping file
        if package_name in mappings:
            channels = mappings[package_name]
            if channels:
                channel_mapping = next(iter(channels.values()))
                if isinstance(channel_mapping, dict):
                    # Platform-specific mapping
                    pixi_to_mapping = {
                        "linux-64": "linux",
                        "linux-aarch64": "linux",
                        "osx-64": "osx",
                        "osx-arm64": "osx",
                        "win-64": "win64",
                    }
                    mapping_platform = pixi_to_mapping.get(platform, "linux")
                    packages = channel_mapping.get(mapping_platform, [])
                    source = PackageSource.MAPPING
                    conda_packages = packages if packages else []
                elif isinstance(channel_mapping, list):
                    source = PackageSource.MAPPING
                    conda_packages = channel_mapping

        # 3. Check if it's in ROS distro
        if source is None and self.has_package(package_name):
            conda_name = package_name.replace("_", "-")
            source = PackageSource.ROS_DISTRO
            conda_packages = [f"ros-{self.distro_name}-{conda_name}"]

        # 4. Check if it's available on conda-forge (without ros-distro prefix)
        if source is None:
            conda_name = package_name.replace("_", "-")
            if self.check_conda_forge_availability(conda_name, platform):
                source = PackageSource.CONDA_FORGE
                conda_packages = [conda_name]

        # 5. Not found
        if source is None:
            print(
                f"Package '{package_name}' not found in workspace, mappings, "
                f"ROS distro, or conda-forge."
            )
            return PackageValidationResult(
                package_name=package_name,
                source=PackageSource.NOT_FOUND,
                conda_packages=[],
                error=f"Package '{package_name}' not found in any source",
            )

        # Step 2: Validate packages exist in their expected channels
        # Note: We don't validate mapped packages - we trust the mappings
        if source == PackageSource.ROS_DISTRO:
            # Validate ROS package exists in robostack channel
            robostack_channel = f"https://prefix.dev/robostack-{self.distro_name}"
            ros_conda_name = conda_packages[0]

            if not self.check_package_availability(
                ros_conda_name, platform, robostack_channel
            ):
                print(
                    f"Package '{package_name}' found in ROS {self.distro_name} "
                    f"distro index but '{ros_conda_name}' not available in "
                    f"robostack-{self.distro_name}."
                )
                # Keep the conda package name so we can show it in NOT_FOUND
                return PackageValidationResult(
                    package_name=package_name,
                    source=PackageSource.NOT_FOUND,
                    conda_packages=[ros_conda_name],
                    error=(
                        f"ROS package not available in robostack-{self.distro_name}"
                    ),
                )

        # Source determined and validated
        return PackageValidationResult(
            package_name=package_name,
            source=source,
            conda_packages=conda_packages,
        )
