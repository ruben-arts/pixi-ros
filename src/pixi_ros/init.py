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


def init_workspace(distro: str, workspace_path: Path | None = None) -> bool:
    """
    Initialize or update pixi.toml for a ROS workspace.

    Args:
        distro: ROS distribution (e.g., "humble", "iron", "jazzy")
        workspace_path: Path to workspace root (defaults to current directory)

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
    _ensure_workspace_section(pixi_config, workspace_path)
    _ensure_channels(pixi_config, distro)
    _ensure_dependencies(pixi_config, packages, distro)
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
            content.append("   Check it out for detailed usage instructions!\n\n", style="dim")

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


def _ensure_workspace_section(config: dict, workspace_path: Path):
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

    # Set platforms if not present
    if "platforms" not in workspace:
        # Only add the current platform by default
        current_platform = str(Platform.current())
        workspace["platforms"] = [current_platform]


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

    availability = {pkg: False for pkg in packages}

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


def _ensure_dependencies(config: dict, packages, distro: str):
    """Ensure all ROS dependencies are present with comments showing source."""
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

    # Collect dependencies from each package
    for pkg in packages:
        for ros_dep in pkg.get_all_dependencies():
            # Skip workspace packages (they're built locally)
            if ros_dep in workspace_pkg_names:
                continue

            # Map to conda packages
            conda_packages = map_ros_to_conda(ros_dep, distro)

            # Skip if no conda packages were returned
            if not conda_packages:
                continue

            for conda_dep in conda_packages:
                if conda_dep:
                    if conda_dep not in dep_sources:
                        dep_sources[conda_dep] = set()
                    dep_sources[conda_dep].add(pkg.name)

    # Expand GL requirements (REQUIRE_GL, REQUIRE_OPENGL) to platform-specific packages
    # This replaces placeholder strings with actual conda packages
    expanded_dep_sources: dict[str, set[str]] = {}
    all_conda_packages = list(dep_sources.keys())
    expanded_packages = expand_gl_requirements(all_conda_packages)

    # Rebuild dep_sources with expanded packages
    for expanded_pkg in expanded_packages:
        # For expanded packages, merge the sources from the placeholder packages
        sources = set()
        for original_pkg, pkg_sources in dep_sources.items():
            if original_pkg == expanded_pkg:
                # Direct match
                sources.update(pkg_sources)
            elif original_pkg in ("REQUIRE_GL", "REQUIRE_OPENGL"):
                # This was a placeholder, include its sources for all expanded packages
                sources.update(pkg_sources)

        if sources:
            expanded_dep_sources[expanded_pkg] = sources

    dep_sources = expanded_dep_sources

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
    if dep_sources:
        dependencies.add(tomlkit.nl())
        dependencies.add(tomlkit.comment("Workspace dependencies"))

        # Get channels and platform for availability checking
        channels = config.get("workspace", {}).get("channels", [])
        current_platform = Platform.current()

        # Get list of packages to check
        packages_to_check = [
            conda_dep
            for conda_dep in dep_sources.keys()
            if conda_dep not in dependencies
        ]

        # Check package availability if channels are configured
        availability = {}
        if channels and packages_to_check:
            typer.echo("Checking package availability in channels...")
            availability = _check_package_availability(
                packages_to_check, channels, current_platform
            )

        # Add all dependencies in alphabetical order
        available_packages = []
        unavailable_packages = []

        for conda_dep in sorted(dep_sources.keys()):
            if conda_dep not in dependencies:
                # Check if we have availability info
                is_available = availability.get(
                    conda_dep, True
                )  # Default to True if not checked

                if is_available:
                    available_packages.append(conda_dep)
                else:
                    unavailable_packages.append(conda_dep)

        # Add available packages
        for conda_dep in available_packages:
            version = dep_versions.get(conda_dep, "*")
            dependencies[conda_dep] = version

        # Add unavailable packages as comments
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


def _ensure_tasks(config: dict):
    """Ensure common ROS tasks are defined."""
    tasks = config.setdefault("tasks", {})

    # Define common ROS tasks if not present
    default_tasks = {
        "build": "colcon build",
        "test": "colcon test",
        "clean": "rm -rf build install log",
    }

    for task_name, task_cmd in default_tasks.items():
        if task_name not in tasks:
            tasks[task_name] = task_cmd

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
