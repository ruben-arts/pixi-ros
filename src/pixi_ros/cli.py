"""Main CLI entry point for pixi-ros."""

from typing import Annotated

import typer

from pixi_ros.init import init_workspace

app = typer.Typer(
    name="pixi-ros",
    help="Pixi extension for ROS package management",
    no_args_is_help=True,
)

pkg_app = typer.Typer(help="Manage ROS packages")
app.add_typer(pkg_app, name="pkg")


@app.command()
def init(
    distro: Annotated[
        str,
        typer.Argument(help="ROS distribution (e.g., humble, iron, jazzy)"),
    ],
):
    """Initialize pixi.toml for a ROS workspace."""
    init_workspace(distro)


@app.command()
def add(
    dependency: Annotated[str, typer.Argument(help="ROS package to add as dependency")],
    build: Annotated[
        bool, typer.Option("--build", help="Add as build dependency")
    ] = False,
    exec: Annotated[
        bool, typer.Option("--exec", help="Add as exec dependency")
    ] = False,
    test: Annotated[
        bool, typer.Option("--test", help="Add as test dependency")
    ] = False,
):
    """Add a dependency to the current ROS package."""
    dep_type = "exec"  # default
    if build:
        dep_type = "build"
    elif test:
        dep_type = "test"

    typer.echo(f"Adding {dependency} as {dep_type} dependency")
    # TODO: Implement in Stage 5
    raise typer.Exit(code=1)


@pkg_app.command()
def create(
    name: Annotated[str, typer.Argument(help="Name of the ROS package to create")],
    python: Annotated[
        bool, typer.Option("--python", help="Create Python package")
    ] = False,
    cpp: Annotated[bool, typer.Option("--cpp", help="Create C++ package")] = False,
):
    """Create a new ROS package."""
    if not python and not cpp:
        typer.echo("Error: Must specify either --python or --cpp", err=True)
        raise typer.Exit(code=1)

    if python and cpp:
        typer.echo("Error: Cannot specify both --python and --cpp", err=True)
        raise typer.Exit(code=1)

    lang = "Python" if python else "C++"
    typer.echo(f"Creating {lang} package: {name}")
    # TODO: Implement in Stage 4
    raise typer.Exit(code=1)


def main():
    """Entry point for the CLI."""
    app()


if __name__ == "__main__":
    main()
