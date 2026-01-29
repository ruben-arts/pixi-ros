"""Initialize pixi.toml for ROS workspaces."""

from pathlib import Path

import tomlkit
import typer
from rattler import Platform

from pixi_ros.mappings import map_ros_to_conda, validate_distro
from pixi_ros.utils import detect_cmake_version_requirement
from pixi_ros.workspace import discover_packages, find_workspace_root


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
        typer.secho(
            f"✓ Successfully initialized {pixi_toml_path}", fg="green", bold=True
        )

        # Display helpful next steps
        typer.echo("")
        typer.secho("Next steps:", fg="cyan", bold=True)
        typer.echo("  1. Install dependencies:  ", nl=False)
        typer.secho("pixi install", fg="yellow")
        typer.echo("  2. Build workspace:       ", nl=False)
        typer.secho("pixi run build", fg="yellow")
        typer.echo("  3. Run tests:             ", nl=False)
        typer.secho("pixi run test", fg="yellow")
        typer.echo("")
        typer.secho("Tip:", fg="blue", bold=True, nl=False)
        typer.echo(" You can add more packages with:")
        typer.echo("     ", nl=False)
        typer.secho("pixi add <conda-package>", fg="yellow")

        return True
    except Exception as e:
        typer.echo(f"Error writing pixi.toml: {e}", err=True)
        raise typer.Exit(code=1) from e


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
            for conda_dep in conda_packages:
                if conda_dep:
                    if conda_dep not in dep_sources:
                        dep_sources[conda_dep] = set()
                    dep_sources[conda_dep].add(pkg.name)

    # Create or get dependencies table
    if "dependencies" not in config:
        config["dependencies"] = tomlkit.table()
    dependencies = config["dependencies"]

    # Add base ROS dependencies with comment
    base_deps = {
        f"ros-{distro}-ros-base": "*",
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

    # Add package dependencies with source comments
    if dep_sources:
        # Group dependencies by their source packages
        # sources_key -> list of conda deps
        groups: dict[str, list[str]] = {}
        for conda_dep, sources in dep_sources.items():
            if conda_dep not in dependencies:
                sources_key = ", ".join(sorted(sources))
                if sources_key not in groups:
                    groups[sources_key] = []
                groups[sources_key].append(conda_dep)

        # Sort groups by collection size (largest first), then alphabetically
        sorted_groups = sorted(
            groups.keys(),
            key=lambda k: (-len(k.split(", ")), k)
        )

        # Add groups in sorted order
        for sources_key in sorted_groups:
            dependencies.add(tomlkit.nl())
            sources_list = sources_key.split(", ")
            if len(sources_list) == 1:
                comment = f"From package: {sources_key}"
            else:
                comment = f"From packages: {sources_key}"
            dependencies.add(tomlkit.comment(comment))

            for conda_dep in sorted(groups[sources_key]):
                # Use version constraint if available, otherwise "*"
                version = dep_versions.get(conda_dep, "*")
                dependencies[conda_dep] = version

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
