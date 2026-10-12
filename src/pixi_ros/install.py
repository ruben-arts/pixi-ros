"""Install ROS workspace dependencies into a native Pixi environment."""

import os
import re
import shutil
import subprocess
from pathlib import Path

import tomlkit
import typer
from rattler import Platform

from pixi_ros.init import update_manifest
from pixi_ros.mappings import get_platforms, get_ros_distros, reload_mappings
from pixi_ros.workspace import discover_packages, find_workspace_root


def _manifest_distros(config: dict) -> set[str]:
    """Find distributions in the default environment, not unrelated features."""
    distros = set()
    workspace = config.get("workspace", config.get("project", {}))
    for channel in workspace.get("channels", []):
        name = channel if isinstance(channel, str) else channel.get("channel", "")
        match = re.search(r"(?:^|/)robostack-([a-z]+)(?:/|$)", name)
        if match:
            distros.add(match[1])
    for section in [config, *config.get("target", {}).values()]:
        for table in ("dependencies", "host-dependencies", "build-dependencies", "dev"):
            for name in section.get(table, {}):
                match = re.match(r"ros-([a-z]+)-", name)
                if match:
                    distros.add(match[1])
        distro = section.get("activation", {}).get("env", {}).get("ROS_DISTRO")
        if distro:
            distros.add(str(distro))
    return distros


def _select_distro(distro: str | None, config: dict) -> str:
    available = get_ros_distros()
    existing = _manifest_distros(config)
    if len(existing) > 1:
        raise typer.BadParameter(
            "pixi.toml configures multiple ROS distributions: "
            + ", ".join(sorted(existing))
            + ". Select a single distribution in the default environment first."
        )
    if distro and existing and distro not in existing:
        raise typer.BadParameter(
            f"--distro {distro} conflicts with pixi.toml ({next(iter(existing))}). "
            "Change the manifest explicitly before switching distributions."
        )
    if distro is None and existing:
        return next(iter(existing))
    if distro is not None:
        if distro not in available:
            raise typer.BadParameter(
                f"Unknown ROS distribution '{distro}'. Choose {', '.join(available)}."
            )
        return distro

    typer.echo("Available ROS distributions:")
    for index, name in enumerate(available, 1):
        typer.echo(f"  {index}. {name}")
    default = os.environ.get("ROS_DISTRO")
    if default not in available:
        default = None
    while True:
        selection = typer.prompt(
            "Select a distribution (number or name)", default=default, type=str
        ).strip()
        if selection in available:
            return selection
        if selection.isdigit() and 1 <= int(selection) <= len(available):
            return available[int(selection) - 1]
        typer.echo(f"Choose one of: {', '.join(available)}", err=True)


def install_workspace(
    distro: str | None = None,
    platforms: list[str] | None = None,
    channels: list[str] | None = None,
) -> None:
    """Prepare the workspace if necessary, then let Pixi solve and install it."""
    pixi = shutil.which("pixi")
    if pixi is None:
        typer.echo(
            "Error: pixi is required to install the environment. "
            "Install it from https://pixi.sh/latest/installation/ and retry.",
            err=True,
        )
        raise typer.Exit(code=1)

    root = find_workspace_root()
    if root is None:
        typer.echo(
            "Error: Could not find a ROS workspace. Run from a directory "
            "containing package.xml files or a pixi.toml.",
            err=True,
        )
        raise typer.Exit(code=1)
    manifest = root / "pixi.toml"
    try:
        config = tomlkit.loads(manifest.read_text()) if manifest.exists() else {}
        packages = discover_packages(root)
    except (OSError, ValueError) as e:
        typer.echo(f"Error reading workspace: {e}", err=True)
        raise typer.Exit(code=1) from e

    if packages or not manifest.exists():
        distro = _select_distro(distro, config)
        workspace = config.get("workspace", config.get("project", {}))
        selected_platforms = list(workspace.get("platforms", []))
        for platform in platforms or []:
            if platform not in get_platforms():
                raise typer.BadParameter(f"Unsupported platform: {platform}")
            if platform not in selected_platforms:
                selected_platforms.append(platform)
        current = str(Platform.current())
        if not selected_platforms:
            selected_platforms = [current]
        elif current not in selected_platforms:
            typer.confirm(
                f"The workspace does not target {current}. "
                "Add it to install on this machine?",
                default=True,
                abort=True,
            )
            selected_platforms.append(current)

        # Workspace-local mappings and relative channels must resolve from the
        # workspace root, even when invoked inside a source package.
        previous = Path.cwd()
        try:
            os.chdir(root)
            reload_mappings()
            update_manifest(
                distro,
                root,
                platforms=selected_platforms,
                channels=channels,
                dev=bool(config.get("dev")),
                strict=True,
            )
        finally:
            os.chdir(previous)
            reload_mappings()

    typer.echo(f"Installing environment from {manifest}")
    try:
        result = subprocess.run(
            [pixi, "install", "--manifest-path", str(manifest)],
            cwd=root,
            check=False,
        )
    except OSError as e:
        typer.echo(f"Error running pixi: {e}", err=True)
        raise typer.Exit(code=1) from e
    if result.returncode:
        raise typer.Exit(code=result.returncode if result.returncode > 0 else 1)
    typer.echo(f"Environment installed at {root}.")
    typer.echo(
        "Run 'pixi shell' to activate it, or 'pixi run build' to build the workspace."
    )
