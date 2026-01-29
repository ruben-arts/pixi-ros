"""Tests for CLI functionality."""

from typer.testing import CliRunner

from pixi_ros.cli import app

runner = CliRunner()


def test_app_help():
    """Test that the main help command works."""
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "pixi-ros" in result.stdout
    assert "Pixi extension for ROS package management" in result.stdout


def test_init_help():
    """Test that init help command works."""
    result = runner.invoke(app, ["init", "--help"])
    assert result.exit_code == 0
    assert "Initialize pixi.toml for a ROS workspace" in result.stdout
    assert "distro" in result.stdout.lower()  # Check for distro argument


def test_no_args_shows_help():
    """Test that running with no args shows help."""
    result = runner.invoke(app, [])
    # no_args_is_help=True causes Typer to show help
    # Exit code may vary, but help should be shown
    output = result.stdout + result.stderr
    assert "pixi-ros" in output or "Usage" in output
