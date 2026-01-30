"""Initialize pixi.toml for ROS workspaces."""

from pathlib import Path

import tomlkit
import typer
from rattler import Channel, Gateway, Platform
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from pixi_ros.mappings import expand_gl_requirements, map_ros_to_conda, validate_distro
from pixi_ros.utils import detect_cmake_version_requirement
from pixi_ros.workspace import discover_packages, find_workspace_root

console = Console()


def init_workspace(
    distro: str,
    workspace_path: Path | None = None,
    platforms: list[str] | None = None,
) -> bool:
    """
    Initialize or update pixi.toml for a ROS workspace.

    Args:
        distro: ROS distribution (e.g., "humble", "iron", "jazzy")
        workspace_path: Path to workspace root (defaults to current directory)
        platforms: Target platforms (e.g., ["linux-64", "osx-arm64"])

    Returns:
        True if successful, False otherwise

    Raises:
        typer.Exit: If validation fails or workspace not found
    """
    # Validate distro
    if not validate_distro(distro):
        typer.echo(f"Error: Unsupported ROS distribution '{distro}'", err=True)
        typer.echo("Supported distros: humble, iron, jazzy, rolling", err=True)
        raise typer.Exit(code=1)

    # Find workspace root
    if workspace_path is None:
        workspace_path = find_workspace_root()
        if workspace_path is None:
            typer.echo(
                "Error: Could not find ROS workspace. "
                "Make sure you're in a directory with ROS packages.",
                err=True,
            )
            raise typer.Exit(code=1)
    else:
        workspace_path = workspace_path.resolve()

    typer.echo(f"Initializing ROS {distro} workspace at: {workspace_path}")

    # Discover packages in workspace
    try:
        packages = discover_packages(workspace_path)
        if not packages:
            typer.echo(
                "Warning: No ROS packages found in workspace. "
                "Creating minimal pixi.toml.",
                err=True,
            )
    except ValueError as e:
        typer.echo(f"Error discovering packages: {e}", err=True)
        raise typer.Exit(code=1) from e

    if packages:
        package_names = ", ".join(p.name for p in packages)
        typer.echo(f"Found {len(packages)} package(s): {package_names}")

    # Load or create pixi.toml
    pixi_toml_path = workspace_path / "pixi.toml"
    if pixi_toml_path.exists():
        typer.echo(f"Updating existing {pixi_toml_path}")
        try:
            with open(pixi_toml_path) as f:
                pixi_config = tomlkit.load(f)
        except Exception as e:
            typer.echo(f"Error reading pixi.toml: {e}", err=True)
            raise typer.Exit(code=1) from e
    else:
        typer.echo(f"Creating new {pixi_toml_path}")
        pixi_config = tomlkit.document()

    # Display discovered dependencies
    _display_dependencies(packages, distro)

    # Update configuration
    _ensure_workspace_section(pixi_config, workspace_path, platforms)
    _ensure_channels(pixi_config, distro)
    _ensure_dependencies(pixi_config, packages, distro, platforms)
    _ensure_tasks(pixi_config)
    _ensure_activation(pixi_config)

    # Write pixi.toml
    try:
        with open(pixi_toml_path, "w") as f:
            tomlkit.dump(pixi_config, f)

        # Create README_PIXI.md to help users
        _create_readme(workspace_path, distro)
        typer.secho(
            f"✓ Successfully initialized {pixi_toml_path}", fg="green", bold=True
        )

        # Inform about README
        readme_path = workspace_path / "README_PIXI.md"
        readme_created = readme_path.exists()

        # Display helpful next steps with Rich
        typer.echo("")

        # Build the content for the panel
        content = Text()

        if readme_created:
            content.append("📖 Created README_PIXI.md\n", style="green bold")
            content.append(
                "   Check it out for detailed usage instructions!\n\n", style="dim"
            )

        content.append("Next steps:\n", style="cyan bold")
        content.append("  1. Install dependencies:  ", style="white")
        content.append("pixi install\n", style="yellow bold")
        content.append("  2. Build workspace:       ", style="white")
        content.append("pixi run build\n", style="yellow bold")
        content.append("  3. Run tests:             ", style="white")
        content.append("pixi run test\n\n", style="yellow bold")

        content.append("💡 Tip: ", style="blue bold")
        content.append("Activate the workspace with ", style="white")
        content.append("pixi shell", style="yellow bold")
        content.append("\n   and then run ROS commands directly.", style="dim")

        # Display in a nice panel
        panel = Panel(
            content,
            title="[bold green]✓ Successfully Initialized[/bold green]",
            border_style="green",
            padding=(1, 2),
        )
        console.print(panel)

        return True
    except Exception as e:
        typer.echo(f"Error writing pixi.toml: {e}", err=True)
        raise typer.Exit(code=1) from e


def _display_dependencies(packages, distro: str):
    """Display discovered dependencies in a rich table."""
    if not packages:
        return

    workspace_pkg_names = {pkg.name for pkg in packages}

    # Collect dependencies organized by ROS package and type
    # Structure: {pkg_name: {dep_type: {ros_dep: [conda_packages]}}}
    pkg_deps: dict[str, dict[str, dict[str, list[str]]]] = {}

    for pkg in packages:
        pkg_deps[pkg.name] = {"Build": {}, "Runtime": {}, "Test": {}}

        # Build dependencies
        for ros_dep in pkg.get_all_build_dependencies():
            if ros_dep in workspace_pkg_names:
                continue
            conda_packages = map_ros_to_conda(ros_dep, distro)
            if conda_packages:
                pkg_deps[pkg.name]["Build"][ros_dep] = conda_packages

        # Runtime dependencies
        for ros_dep in pkg.get_all_runtime_dependencies():
            if ros_dep in workspace_pkg_names:
                continue
            conda_packages = map_ros_to_conda(ros_dep, distro)
            if conda_packages:
                pkg_deps[pkg.name]["Runtime"][ros_dep] = conda_packages

        # Test dependencies
        for ros_dep in pkg.get_all_test_dependencies():
            if ros_dep in workspace_pkg_names:
                continue
            conda_packages = map_ros_to_conda(ros_dep, distro)
            if conda_packages:
                pkg_deps[pkg.name]["Test"][ros_dep] = conda_packages

    # Check if any external dependencies exist
    has_deps = any(
        any(pkg_deps[pkg_name][dep_type] for dep_type in ["Build", "Runtime", "Test"])
        for pkg_name in pkg_deps
    )

    if not has_deps:
        console.print("\n[yellow]No external dependencies found[/yellow]")
        return

    console.print("")

    # Display one table per ROS package with all dependency types
    for pkg_name in sorted(pkg_deps.keys()):
        pkg_info = pkg_deps[pkg_name]

        # Collect all dependencies for this package across all types
        all_deps = []
        for dep_type in ["Build", "Runtime", "Test"]:
            for ros_dep in sorted(pkg_info[dep_type].keys()):
                conda_pkgs = pkg_info[dep_type][ros_dep]
                conda_str = ", ".join(conda_pkgs)
                all_deps.append((ros_dep, dep_type, conda_str))

        # Skip packages with no external dependencies
        if not all_deps:
            continue

        # Create table for this package
        table = Table(
            title=f"Package: {pkg_name}",
            show_header=True,
            header_style="bold cyan",
        )
        table.add_column("ROS Dependency", style="yellow")
        table.add_column("Type", style="blue")
        table.add_column("Conda Packages", style="green")

        # Add all dependencies for this package
        for ros_dep, dep_type, conda_str in all_deps:
            table.add_row(ros_dep, dep_type, conda_str)

        console.print(table)
        console.print("")


def _ensure_workspace_section(
    config: dict, workspace_path: Path, platforms: list[str] | None = None
):
    """Ensure workspace section exists with basic config."""
    if "workspace" not in config:
        config["workspace"] = {}

    workspace = config["workspace"]

    # Set name if not present
    if "name" not in workspace:
        workspace["name"] = workspace_path.name

    # Set channels if not present
    if "channels" not in workspace:
        workspace["channels"] = []

    # Set or extend platforms
    if "platforms" not in workspace:
        if platforms:
            # Platforms are already in pixi format (linux-64, osx-64, etc.)
            workspace["platforms"] = platforms
        else:
            # Only add the current platform by default
            current_platform = str(Platform.current())
            workspace["platforms"] = [current_platform]
    elif platforms:
        # Extend existing platforms list with new ones (avoiding duplicates)
        existing_platforms = workspace["platforms"]
        if not isinstance(existing_platforms, list):
            existing_platforms = [existing_platforms]

        # Add new platforms that aren't already in the list
        for platform in platforms:
            if platform not in existing_platforms:
                existing_platforms.append(platform)

        workspace["platforms"] = existing_platforms


def _ensure_channels(config: dict, distro: str):
    """Ensure required ROS channels are present."""
    workspace = config.setdefault("workspace", {})
    channels = workspace.setdefault("channels", [])

    # Required channels for ROS
    channel_host = "https://prefix.dev"
    required_channels = [
        f"{channel_host}/robostack-{distro}",
        f"{channel_host}/conda-forge",
    ]

    for channel in required_channels:
        if channel not in channels:
            channels.append(channel)

    workspace["channels"] = channels


def _check_package_availability(
    packages: list[str], channels: list[str], platform: Platform
) -> dict[str, bool]:
    """
    Check if packages are available in the given channels.

    Args:
        packages: List of conda package names to check
        channels: List of channel URLs
        platform: Platform to check for

    Returns:
        Dictionary mapping package names to availability (True/False)
    """
    import asyncio

    availability = dict.fromkeys(packages, False)

    try:
        # Create gateway for fetching repo data
        gateway = Gateway()

        # Convert channel URLs to Channel objects
        channel_objects = [Channel(url) for url in channels]

        # Query all channels at once (gateway.query is async)
        repo_data_by_channel = asyncio.run(
            gateway.query(
                channel_objects,
                [platform],
                specs=packages,  # Correct parameter name
                recursive=False,  # Don't fetch dependencies
            )
        )

        # repo_data_by_channel is a list of lists (one per channel)
        # Check all channels for each package
        for channel_records in repo_data_by_channel:
            for record in channel_records:
                # Check if any of our packages match this record
                for package_name in packages:
                    if record.name.normalized == package_name.lower():
                        availability[package_name] = True

    except Exception as e:
        # If query fails, log the error but continue (all marked as unavailable)
        console.print(
            f"[yellow]Warning: Could not check package availability: {e}[/yellow]"
        )

    return availability


def _ensure_dependencies(
    config: dict, packages, distro: str, platforms: list[str] | None = None
):
    """
    Ensure all ROS dependencies are present with comments showing source.

    Generates platform-specific dependencies if multiple platforms are specified.
    Common dependencies (available on all platforms) go in [dependencies],
    platform-specific ones go in [target.{platform}.dependencies].
    """
    # Default to current platform if none specified
    if not platforms:
        platforms = [str(Platform.current())]
    # Track which packages depend on which conda packages
    # conda_dep -> set of package names
    dep_sources: dict[str, set[str]] = {}
    # Track version constraints for special dependencies
    dep_versions: dict[str, str] = {}
    workspace_pkg_names = {pkg.name for pkg in packages}

    # Detect special version requirements from package files
    for pkg in packages:
        # Check for cmake version requirements
        pkg_path = pkg.path.parent
        cmake_version = detect_cmake_version_requirement(pkg_path)
        if cmake_version:
            dep_versions["cmake"] = cmake_version

        # Collect version constraints from package.xml
        for ros_dep, version_constraint in pkg.dependency_versions.items():
            # Skip workspace packages
            if ros_dep in workspace_pkg_names:
                continue

            # Map ROS package to conda packages
            # Note: We use the first platform for mapping since version constraints
            # should be the same across platforms for a given ROS package
            conda_packages = map_ros_to_conda(ros_dep, distro)

            # Apply version constraint to all mapped conda packages
            for conda_dep in conda_packages:
                if conda_dep and not conda_dep.startswith("REQUIRE_"):
                    # If package already has a constraint, combine them
                    if conda_dep in dep_versions:
                        dep_versions[conda_dep] = (
                            f"{dep_versions[conda_dep]},{version_constraint}"
                        )
                    else:
                        dep_versions[conda_dep] = version_constraint

    # Platforms come from CLI as pixi platform names (linux-64, osx-64, etc.)
    # Map them to mapping platform names for querying the mapping files
    pixi_to_mapping = {
        "linux-64": "linux",
        "osx-64": "osx",
        "osx-arm64": "osx",
        "win-64": "win64",
    }

    # Group pixi platforms by their mapping platform
    # This way osx-64 and osx-arm64 share the same dependencies
    platform_groups = {}
    for platform in platforms:
        mapping_platform = pixi_to_mapping.get(platform, "linux")
        if mapping_platform not in platform_groups:
            platform_groups[mapping_platform] = []
        platform_groups[mapping_platform].append(platform)

    # Collect dependencies per mapping platform (which groups similar pixi platforms)
    # Structure: mapping_platform -> conda_dep -> set of ROS package names
    platform_deps: dict[str, dict[str, set[str]]] = {
        mapping_platform: {} for mapping_platform in platform_groups.keys()
    }

    # Collect dependencies from each package, mapped for each platform
    for mapping_platform in platform_groups.keys():
        for pkg in packages:
            for ros_dep in pkg.get_all_dependencies():
                # Skip workspace packages (they're built locally)
                if ros_dep in workspace_pkg_names:
                    continue

                # Map to conda packages for this mapping platform
                conda_packages = map_ros_to_conda(
                    ros_dep, distro, platform_override=mapping_platform
                )

                # Skip if no conda packages were returned
                if not conda_packages:
                    continue

                for conda_dep in conda_packages:
                    if conda_dep:
                        if conda_dep not in platform_deps[mapping_platform]:
                            platform_deps[mapping_platform][conda_dep] = set()
                        platform_deps[mapping_platform][conda_dep].add(pkg.name)

        # Expand GL requirements for this mapping platform
        all_conda_packages = list(platform_deps[mapping_platform].keys())
        expanded_packages = expand_gl_requirements(
            all_conda_packages, platform_override=mapping_platform
        )

        # Rebuild platform deps with expanded packages
        expanded_platform_deps: dict[str, set[str]] = {}
        for expanded_pkg in expanded_packages:
            sources = set()
            for original_pkg, pkg_sources in platform_deps[mapping_platform].items():
                if original_pkg == expanded_pkg:
                    sources.update(pkg_sources)
                elif original_pkg in ("REQUIRE_GL", "REQUIRE_OPENGL"):
                    sources.update(pkg_sources)

            if sources:
                expanded_platform_deps[expanded_pkg] = sources

        platform_deps[mapping_platform] = expanded_platform_deps

    # Determine common dependencies (present in all mapping platforms)
    mapping_platform_list = list(platform_groups.keys())
    if len(mapping_platform_list) > 1:
        all_deps = set(platform_deps[mapping_platform_list[0]].keys())
        for mapping_platform in mapping_platform_list[1:]:
            all_deps &= set(platform_deps[mapping_platform].keys())
        common_deps = all_deps
    else:
        # Single mapping platform - all deps are "common"
        common_deps = set(platform_deps[mapping_platform_list[0]].keys())

    # For backwards compatibility when single platform, use old behavior
    dep_sources = (
        platform_deps[mapping_platform_list[0]]
        if len(mapping_platform_list) == 1
        else {
            dep: platform_deps[mapping_platform_list[0]][dep]
            for dep in common_deps
            if dep in platform_deps[mapping_platform_list[0]]
        }
    )

    # Create or get dependencies table
    if "dependencies" not in config:
        config["dependencies"] = tomlkit.table()
    dependencies = config["dependencies"]

    # Add base ROS dependencies with comment
    base_deps = {
        f"ros-{distro}-ros-base": "*",
        "pkg-config": "*",
        "compilers": "*",
        "make": "*",
        "ninja": "*",
    }

    # Add newline and comment before base deps if dependencies table is empty
    if len(dependencies) == 0:
        dependencies.add(tomlkit.comment("Base ROS dependencies"))

    for dep, version in base_deps.items():
        if dep not in dependencies:
            dependencies[dep] = version

    # Add ros2cli packages
    ros2cli_deps = map_ros_to_conda("ros2cli", distro)
    if ros2cli_deps:
        for conda_pkg in ros2cli_deps:
            if conda_pkg and conda_pkg not in dependencies:
                dependencies[conda_pkg] = "*"

    # Add build tools
    if "colcon-common-extensions" not in dependencies:
        dependencies.add(tomlkit.nl())
        dependencies.add(tomlkit.comment("Build tools"))
        dependencies["colcon-common-extensions"] = "*"

    # Add cmake with version constraint if detected
    if "cmake" in dep_versions and "cmake" not in dependencies:
        dependencies["cmake"] = dep_versions["cmake"]

    # Add package dependencies
    channels = config.get("workspace", {}).get("channels", [])

    # Add common dependencies (available on all platforms)
    if dep_sources:
        dependencies.add(tomlkit.nl())
        if len(mapping_platform_list) > 1:
            dependencies.add(
                tomlkit.comment("Workspace dependencies (common across platforms)")
            )
        else:
            dependencies.add(tomlkit.comment("Workspace dependencies"))

        # For common deps, check on first pixi platform as representative
        first_pixi_platform = platforms[0]
        first_platform = Platform(first_pixi_platform)

        packages_to_check = [
            conda_dep
            for conda_dep in dep_sources.keys()
            if conda_dep not in dependencies
        ]

        availability = {}
        if channels and packages_to_check:
            typer.echo(
                f"Checking common package availability for {first_pixi_platform}..."
            )
            availability = _check_package_availability(
                packages_to_check, channels, first_platform
            )

        available_packages = []
        unavailable_packages = []

        for conda_dep in sorted(dep_sources.keys()):
            if conda_dep not in dependencies:
                is_available = availability.get(conda_dep, True)
                if is_available:
                    available_packages.append(conda_dep)
                else:
                    unavailable_packages.append(conda_dep)

        for conda_dep in available_packages:
            version = dep_versions.get(conda_dep, "*")
            dependencies[conda_dep] = version

        if unavailable_packages:
            dependencies.add(tomlkit.nl())
            dependencies.add(
                tomlkit.comment(
                    "The following packages were not found in the configured channels:"
                )
            )
            for conda_dep in unavailable_packages:
                version = dep_versions.get(conda_dep, "*")
                dependencies.add(
                    tomlkit.comment(f'{conda_dep} = "{version}"  # NOT FOUND')
                )

    config["dependencies"] = dependencies

    # Add platform-specific dependencies if multiple mapping platforms
    # First, identify unix dependencies (available on both linux and osx, but not win)
    unix_deps = {}
    if len(mapping_platform_list) > 1:
        has_linux = "linux" in mapping_platform_list
        has_osx = "osx" in mapping_platform_list
        has_win = "win64" in mapping_platform_list or "win" in mapping_platform_list

        if has_linux and has_osx:
            # Find deps that are on both linux and osx
            linux_only = set(platform_deps.get("linux", {}).keys())
            osx_only = set(platform_deps.get("osx", {}).keys())
            unix_candidates = (linux_only & osx_only) - common_deps

            # If we also have windows, only move to unix if NOT on windows
            if has_win:
                win_deps = set(platform_deps.get("win64", {}).keys()) | set(
                    platform_deps.get("win", {}).keys()
                )
                unix_deps_keys = unix_candidates - win_deps
            else:
                unix_deps_keys = unix_candidates

            # Move to unix section
            for dep in unix_deps_keys:
                if dep in platform_deps.get("linux", {}):
                    unix_deps[dep] = platform_deps["linux"][dep]

        # Add unix section if there are unix-specific dependencies
        if unix_deps:
            if "target" not in config:
                config["target"] = tomlkit.table()
            if "unix" not in config["target"]:
                config["target"]["unix"] = tomlkit.table()
            if "dependencies" not in config["target"]["unix"]:
                config["target"]["unix"]["dependencies"] = tomlkit.table()

            target_deps = config["target"]["unix"]["dependencies"]

            if len(target_deps) == 0:
                target_deps.add(
                    tomlkit.comment("Unix-specific dependencies (Linux and macOS)")
                )

            # Check availability on linux platform as representative
            representative_pixi_platform = platform_groups.get(
                "linux", platform_groups.get("osx", platforms)
            )[0]
            platform_obj = Platform(representative_pixi_platform)
            packages_to_check = list(unix_deps.keys())

            availability = {}
            if channels and packages_to_check:
                typer.echo("Checking package availability for unix...")
                availability = _check_package_availability(
                    packages_to_check, channels, platform_obj
                )

            available_packages = []
            unavailable_packages = []

            for conda_dep in sorted(unix_deps.keys()):
                if conda_dep not in target_deps:
                    is_available = availability.get(conda_dep, True)
                    if is_available:
                        available_packages.append(conda_dep)
                    else:
                        unavailable_packages.append(conda_dep)

            for conda_dep in available_packages:
                version = dep_versions.get(conda_dep, "*")
                target_deps[conda_dep] = version

            if unavailable_packages:
                target_deps.add(tomlkit.nl())
                target_deps.add(
                    tomlkit.comment("The following packages were not found:")
                )
                for conda_dep in unavailable_packages:
                    version = dep_versions.get(conda_dep, "*")
                    target_deps.add(
                        tomlkit.comment(f'{conda_dep} = "{version}"  # NOT FOUND')
                    )

    # Now add remaining platform-specific dependencies (not in common, not in unix)
    if len(mapping_platform_list) > 1:
        for mapping_platform in mapping_platform_list:
            platform_specific_deps = {
                dep: sources
                for dep, sources in platform_deps[mapping_platform].items()
                if dep not in common_deps and dep not in unix_deps
            }

            if platform_specific_deps:
                # Create target section if needed
                if "target" not in config:
                    config["target"] = tomlkit.table()
                if mapping_platform not in config["target"]:
                    config["target"][mapping_platform] = tomlkit.table()
                if "dependencies" not in config["target"][mapping_platform]:
                    config["target"][mapping_platform]["dependencies"] = tomlkit.table()

                target_deps = config["target"][mapping_platform]["dependencies"]

                # Add comment
                if len(target_deps) == 0:
                    target_deps.add(
                        tomlkit.comment(
                            f"Platform-specific dependencies for {mapping_platform}"
                        )
                    )

                # Check availability for this mapping platform
                # Use the first pixi platform in the group as representative
                representative_pixi_platform = platform_groups[mapping_platform][0]
                platform_obj = Platform(representative_pixi_platform)
                packages_to_check = list(platform_specific_deps.keys())

                availability = {}
                if channels and packages_to_check:
                    typer.echo(
                        f"Checking package availability for {mapping_platform}..."
                    )
                    availability = _check_package_availability(
                        packages_to_check, channels, platform_obj
                    )

                available_packages = []
                unavailable_packages = []

                for conda_dep in sorted(platform_specific_deps.keys()):
                    if conda_dep not in target_deps:
                        is_available = availability.get(conda_dep, True)
                        if is_available:
                            available_packages.append(conda_dep)
                        else:
                            unavailable_packages.append(conda_dep)

                for conda_dep in available_packages:
                    version = dep_versions.get(conda_dep, "*")
                    target_deps[conda_dep] = version

                if unavailable_packages:
                    target_deps.add(tomlkit.nl())
                    target_deps.add(
                        tomlkit.comment("The following packages were not found:")
                    )
                    for conda_dep in unavailable_packages:
                        version = dep_versions.get(conda_dep, "*")
                        target_deps.add(
                            tomlkit.comment(f'{conda_dep} = "{version}"  # NOT FOUND')
                        )


def _ensure_tasks(config: dict):
    """Ensure common ROS tasks are defined."""
    tasks = config.setdefault("tasks", {})

    # Define common ROS tasks if not present
    default_tasks = {
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

    for task_name, task_config in default_tasks.items():
        if task_name not in tasks:
            # Create inline table for task configuration
            task_table = tomlkit.inline_table()
            task_table["cmd"] = task_config["cmd"]
            task_table["description"] = task_config["description"]
            tasks[task_name] = task_table

    config["tasks"] = tasks


def _ensure_activation(config: dict):
    """Ensure activation section exists to source ROS setup."""
    if "activation" not in config:
        # Add comment before the activation section
        config.add(tomlkit.nl())
        config.add(
            tomlkit.comment(
                "Scripts to source on environment activation, "
                "found after first colcon build."
            )
        )
        config["activation"] = tomlkit.table()
        config["activation"]["scripts"] = ["install/setup.bash"]
    elif "scripts" not in config["activation"]:
        # Just add scripts if activation exists but scripts don't
        config["activation"]["scripts"] = ["install/setup.bash"]


def _create_readme(workspace_path: Path, distro: str):
    """Create README_PIXI.md to help users understand the pixi-ros workflow."""
    readme_path = workspace_path / "README_PIXI.md"

    # Don't overwrite existing README_PIXI.md
    if readme_path.exists():
        return

    try:
        # Load template from data directory
        template_path = Path(__file__).parent / "data" / "README_PIXI.md.template"
        with open(template_path) as f:
            template_content = f.read()

        # Format template with distro
        readme_content = template_content.format(distro=distro)

        # Write to workspace
        with open(readme_path, "w") as f:
            f.write(readme_content)
    except Exception as e:
        # Don't fail if we can't create the README
        typer.echo(f"Warning: Could not create README_PIXI.md: {e}", err=True)
