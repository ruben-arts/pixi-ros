"""Tests for configuration handling."""

from pathlib import Path
from tempfile import TemporaryDirectory

from pixi_ros.config import find_config_file


def test_find_config_file_in_current_dir():
    """Test finding config file in current directory."""
    with TemporaryDirectory() as tmpdir:
        tmppath = Path(tmpdir).resolve()
        config_path = tmppath / ".pixi-ros.toml"
        config_path.touch()

        result = find_config_file(tmppath)
        assert result is not None
        assert result.resolve() == config_path.resolve()


def test_find_config_file_in_parent_dir():
    """Test finding config file in parent directory."""
    with TemporaryDirectory() as tmpdir:
        tmppath = Path(tmpdir).resolve()
        config_path = tmppath / ".pixi-ros.toml"
        config_path.touch()

        subdir = tmppath / "subdir"
        subdir.mkdir()

        result = find_config_file(subdir)
        assert result is not None
        assert result.resolve() == config_path.resolve()


def test_find_config_file_not_found():
    """Test that None is returned when config file is not found."""
    with TemporaryDirectory() as tmpdir:
        tmppath = Path(tmpdir)
        result = find_config_file(tmppath)
        assert result is None
