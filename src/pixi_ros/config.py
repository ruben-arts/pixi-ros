"""Configuration handling for pixi-ros."""

from pathlib import Path


def find_config_file(start_path: Path | None = None) -> Path | None:
    """
    Search for .pixi-ros.toml configuration file.

    Searches upward from start_path (or cwd) until a config file is found.

    Args:
        start_path: Starting directory for search (defaults to cwd)

    Returns:
        Path to config file if found, None otherwise
    """
    if start_path is None:
        start_path = Path.cwd()

    current = start_path.resolve()

    # Search upward until we hit the root
    while True:
        config_path = current / ".pixi-ros.toml"
        if config_path.exists():
            return config_path

        parent = current.parent
        if parent == current:  # Reached root
            break
        current = parent

    return None
