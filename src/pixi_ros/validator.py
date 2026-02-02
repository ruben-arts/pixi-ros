"""ROS package validation logic."""

import asyncio
from dataclasses import dataclass
from enum import Enum
from functools import lru_cache

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

    @lru_cache(maxsize=128)
    def check_conda_forge_availability(
        self, package_name: str, platform: str
    ) -> bool:
        """
        Check if package is available on conda-forge.

        Args:
            package_name: Conda package name (without ros-distro- prefix)
            platform: Platform string (e.g., "linux-64", "osx-arm64")

        Returns:
            True if package is available on conda-forge
        """
        try:
            gateway = Gateway()
            channel = Channel("https://prefix.dev/conda-forge")
            platform_obj = Platform(platform)

            # Query with 5 second timeout
            repo_data = asyncio.wait_for(
                gateway.query(
                    [channel],
                    [platform_obj],
                    specs=[package_name],
                    recursive=False,
                ),
                timeout=5.0,
            )

            # Check if any records match
            results = asyncio.run(repo_data)
            for channel_records in results:
                for record in channel_records:
                    if record.name.normalized == package_name.lower():
                        return True

            return False
        except (asyncio.TimeoutError, Exception):
            # On error or timeout, assume not available
            return False

    def validate_package(
        self,
        package_name: str,
        workspace_packages: set[str],
        mappings: dict[str, dict[str, list[str] | dict[str, list[str]]]],
        platform: str = "linux-64",
    ) -> PackageValidationResult:
        """
        Validate a ROS package and determine its source.

        Validation priority order:
        1. Workspace packages (local source) → Skip
        2. Mapping file → Use mapped conda packages
        3. ROS distro → Use ros-{distro}-{package}
        4. conda-forge → Use conda package (no prefix)
        5. Not found → Mark as NOT_FOUND

        Args:
            package_name: ROS package name
            workspace_packages: Set of package names in the workspace
            mappings: Package mappings from mapping files
            platform: Target platform (default: "linux-64")

        Returns:
            PackageValidationResult with source and conda package names
        """
        # 1. Check if it's a workspace package
        if package_name in workspace_packages:
            return PackageValidationResult(
                package_name=package_name,
                source=PackageSource.WORKSPACE,
                conda_packages=[],
            )

        # 2. Check if it's in the mapping file
        if package_name in mappings:
            # Get mapped packages (using existing logic from mappings.py)
            channels = mappings[package_name]
            if channels:
                channel_mapping = next(iter(channels.values()))
                if isinstance(channel_mapping, dict):
                    # Platform-specific mapping
                    # Convert pixi platform to mapping platform
                    pixi_to_mapping = {
                        "linux-64": "linux",
                        "osx-64": "osx",
                        "osx-arm64": "osx",
                        "win-64": "win64",
                    }
                    mapping_platform = pixi_to_mapping.get(platform, "linux")
                    packages = channel_mapping.get(mapping_platform, [])
                    return PackageValidationResult(
                        package_name=package_name,
                        source=PackageSource.MAPPING,
                        conda_packages=packages if packages else [],
                    )
                elif isinstance(channel_mapping, list):
                    return PackageValidationResult(
                        package_name=package_name,
                        source=PackageSource.MAPPING,
                        conda_packages=channel_mapping,
                    )

        # 3. Check if it's in ROS distro
        if self.has_package(package_name):
            conda_name = package_name.replace("_", "-")
            return PackageValidationResult(
                package_name=package_name,
                source=PackageSource.ROS_DISTRO,
                conda_packages=[f"ros-{self.distro_name}-{conda_name}"],
            )

        # 4. Check if it's available on conda-forge (without ros-distro prefix)
        conda_name = package_name.replace("_", "-")
        if self.check_conda_forge_availability(conda_name, platform):
            return PackageValidationResult(
                package_name=package_name,
                source=PackageSource.CONDA_FORGE,
                conda_packages=[conda_name],
            )

        # 5. Not found
        return PackageValidationResult(
            package_name=package_name,
            source=PackageSource.NOT_FOUND,
            conda_packages=[],
            error=f"Package '{package_name}' not found in any source",
        )
