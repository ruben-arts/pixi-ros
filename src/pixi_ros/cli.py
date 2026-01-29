"""Main CLI entry point for pixi-ros."""

from typing import Annotated

import typer

from pixi_ros.init import init_workspace
from pixi_ros.mappings import get_ros_distros

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
        str | None,
        typer.Option(
            "--distro",
            "-d",
            help="ROS distribution (e.g., humble, iron, jazzy)",
        ),
    ] = None,
):
    """Initialize pixi.toml for a ROS workspace."""
    # If distro not provided, prompt user to select one
    if distro is None:
        available_distros = get_ros_distros()
        typer.echo("Available ROS distributions:")
        for i, d in enumerate(available_distros, 1):
            typer.echo(f"  {i}. {d}")

        # Prompt for selection
        selection = typer.prompt(
            "\nSelect a distribution (enter number or name)",
            type=str,
        )

        # Parse selection (either number or name)
        try:
            selection_num = int(selection)
            if 1 <= selection_num <= len(available_distros):
                distro = available_distros[selection_num - 1]
            else:
                typer.echo(f"Error: Invalid selection. Please choose 1-{len(available_distros)}", err=True)
                raise typer.Exit(code=1)
        except ValueError:
            # User entered a name instead of number
            if selection in available_distros:
                distro = selection
            else:
                typer.echo(f"Error: '{selection}' is not a valid ROS distribution", err=True)
                typer.echo(f"Available: {', '.join(available_distros)}", err=True)
                raise typer.Exit(code=1)

    init_workspace(distro)


def main():
    """Entry point for the CLI."""
    app()


if __name__ == "__main__":
    main()
