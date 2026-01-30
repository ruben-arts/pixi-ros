"""Main CLI entry point for pixi-ros."""

from typing import Annotated

import typer

from pixi_ros.init import init_workspace
from pixi_ros.mappings import get_platforms, get_ros_distros

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
    platforms: Annotated[
        list[str] | None,
        typer.Option(
            "--platform",
            "-p",
            help="Target platforms (e.g., linux-64, osx-arm64, win-64)."
            " Can be specified multiple times.",
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
            dist_count = len(available_distros)
            if 1 <= selection_num <= dist_count:
                distro = available_distros[selection_num - 1]
            else:
                typer.echo(
                    f"Error: Invalid selection. Please choose 1-{dist_count}",
                    err=True,
                )
                raise typer.Exit(code=1)
        except ValueError as err:
            # User entered a name instead of number
            if selection in available_distros:
                distro = selection
            else:
                typer.echo(
                    f"Error: '{selection}' is not a valid ROS distribution", err=True
                )
                typer.echo(f"Available: {', '.join(available_distros)}", err=True)
                raise typer.Exit(code=1) from err

    # If platforms not provided, prompt user to select
    if platforms is None or len(platforms) == 0:
        available_platforms = get_platforms()
        typer.echo("\nAvailable target platforms:")
        for i, p in enumerate(available_platforms, 1):
            typer.echo(f"  {i}. {p}")

        # Prompt for selection (can be comma-separated or space-separated)
        selection = typer.prompt(
            "\nSelect platforms (enter numbers or names, comma or space separated)",
            type=str,
        )

        # Parse selection (can be numbers or names, comma or space separated)
        platforms = []
        # Split by comma or space
        selections = selection.replace(",", " ").split()

        for sel in selections:
            sel = sel.strip()
            if not sel:
                continue

            try:
                # Try parsing as number
                sel_num = int(sel)
                if 1 <= sel_num <= len(available_platforms):
                    platforms.append(available_platforms[sel_num - 1])
                else:
                    typer.echo(
                        f"Error: Invalid selection {sel_num}."
                        + f"Please choose 1-{len(available_platforms)}",
                        err=True,
                    )
                    raise typer.Exit(code=1)
            except ValueError as err:
                # User entered a name instead of number
                if sel in available_platforms:
                    platforms.append(sel)
                else:
                    typer.echo(f"Error: '{sel}' is not a valid platform", err=True)
                    typer.echo(f"Available: {', '.join(available_platforms)}", err=True)
                    raise typer.Exit(code=1) from err

        if not platforms:
            typer.echo("Error: No platforms selected", err=True)
            raise typer.Exit(code=1)

    init_workspace(distro, platforms=platforms)


def main():
    """Entry point for the CLI."""
    app()


if __name__ == "__main__":
    main()
