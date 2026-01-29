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


def test_add_help():
    """Test that add help command works."""
    result = runner.invoke(app, ["add", "--help"])
    assert result.exit_code == 0
    assert "Add a dependency to the current ROS package" in result.stdout
    assert "--build" in result.stdout
    assert "--exec" in result.stdout
    assert "--test" in result.stdout


def test_pkg_create_help():
    """Test that pkg create help command works."""
    result = runner.invoke(app, ["pkg", "create", "--help"])
    assert result.exit_code == 0
    assert "Create a new ROS package" in result.stdout
    assert "--python" in result.stdout
    assert "--cpp" in result.stdout


def test_pkg_create_requires_language():
    """Test that pkg create requires either --python or --cpp."""
    result = runner.invoke(app, ["pkg", "create", "test_package"])
    assert result.exit_code == 1
    # Error messages go to stdout with Typer's echo
    output = result.stdout + result.stderr
    assert "Must specify either --python or --cpp" in output


def test_pkg_create_rejects_both_languages():
    """Test that pkg create rejects both --python and --cpp."""
    result = runner.invoke(app, ["pkg", "create", "test_package", "--python", "--cpp"])
    assert result.exit_code == 1
    # Error messages go to stdout with Typer's echo
    output = result.stdout + result.stderr
    assert "Cannot specify both --python and --cpp" in output


def test_no_args_shows_help():
    """Test that running with no args shows help."""
    result = runner.invoke(app, [])
    # no_args_is_help=True causes Typer to show help
    # Exit code may vary, but help should be shown
    output = result.stdout + result.stderr
    assert "pixi-ros" in output or "Usage" in output
