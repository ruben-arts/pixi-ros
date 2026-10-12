"""Workspace discovery and management for ROS packages."""

from collections.abc import Iterator
from pathlib import Path

import pathspec

from pixi_ros.package_xml import PackageXML

_SKIP_DIRS = {"build", "install", "log", ".pixi", "tests", "test"}


def load_gitignore_spec(workspace_root: Path) -> pathspec.PathSpec | None:
    """
    Load gitignore patterns from workspace root.

    Args:
        workspace_root: Root directory of the workspace

    Returns:
        PathSpec object with gitignore patterns, or None if no .gitignore exists
    """
    gitignore_path = workspace_root / ".gitignore"
    if not gitignore_path.exists():
        return None

    try:
        with open(gitignore_path) as f:
            patterns = f.read().splitlines()
        return pathspec.PathSpec.from_lines("gitignore", patterns)
    except (OSError, ValueError):
        return None


def find_package_xml(start_path: Path | None = None) -> Path | None:
    """
    Find the nearest package.xml file by searching upward from start_path.

    Args:
        start_path: Starting directory for search (defaults to cwd)

    Returns:
        Path to package.xml if found, None otherwise
    """
    if start_path is None:
        start_path = Path.cwd()

    current = start_path.resolve()

    # Search upward until we hit the root
    while True:
        package_xml = current / "package.xml"
        if package_xml.exists():
            return package_xml

        parent = current.parent
        if parent == current:  # Reached root
            break
        current = parent

    return None


def find_workspace_root(start_path: Path | None = None) -> Path | None:
    """Find a manifest or ROS workspace without searching ancestor trees.

    A recognizable ``src`` workspace takes precedence over unrelated ancestor
    manifests. Within that boundary, the nearest manifest wins. Without either,
    use the enclosing package itself or the package-bearing starting directory.
    """
    current = (start_path if start_path is not None else Path.cwd()).resolve()
    if not current.is_dir():
        return None

    ancestors = (current, *current.parents)
    manifest_root = next(
        (path for path in ancestors if (path / "pixi.toml").is_file()), None
    )
    if manifest_root == current:
        return current

    package_xml = find_package_xml(current)
    package_root = package_xml.parent if package_xml else None
    # A package's own source-code directory is not a workspace boundary.
    workspace_ancestors = package_root.parents if package_root else ancestors
    src_dir = next((path for path in workspace_ancestors if path.name == "src"), None)
    ros_root = None
    has_packages = package_root is not None
    if package_root:
        if src_dir:
            ros_root = src_dir.parent
    else:
        # Only descend into the invocation directory, never an ancestor that
        # might contain unrelated sibling repositories or packages.
        for package_path in _iter_package_xml(current):
            has_packages = True
            if src_dir:
                ros_root = src_dir.parent
                break
            if package_path.is_relative_to(current / "src"):
                ros_root = current
                break

    if ros_root and (
        manifest_root is None or not manifest_root.is_relative_to(ros_root)
    ):
        return ros_root
    if manifest_root:
        return manifest_root
    if package_root:
        return package_root
    return current if has_packages else None


def _iter_package_xml(workspace_root: Path) -> Iterator[Path]:
    """Walk package markers using the same exclusions for root and discovery."""
    gitignore_spec = load_gitignore_spec(workspace_root)

    def _walk(directory: Path) -> Iterator[Path]:
        if (directory / "COLCON_IGNORE").exists():
            return

        package_xml_path = directory / "package.xml"
        if package_xml_path.is_file():
            relative_path = package_xml_path.relative_to(workspace_root)
            if not (gitignore_spec and gitignore_spec.match_file(str(relative_path))):
                yield package_xml_path

        for child in sorted(directory.iterdir()):
            if not child.is_dir():
                continue
            if child.name.startswith(".") or child.name in _SKIP_DIRS:
                continue
            relative_child = child.relative_to(workspace_root)
            if gitignore_spec and gitignore_spec.match_file(str(relative_child)):
                continue
            yield from _walk(child)

    yield from _walk(workspace_root)


def discover_packages(workspace_root: Path) -> list[PackageXML]:
    """
    Discover all ROS packages in a workspace.

    Recursively searches for all package.xml files in the workspace,
    excluding hidden directories (those starting with a dot).

    Args:
        workspace_root: Root directory of the workspace

    Returns:
        List of parsed PackageXML objects

    Raises:
        ValueError: If workspace_root doesn't exist or isn't a directory
    """
    if not workspace_root.exists():
        raise ValueError(f"Workspace root does not exist: {workspace_root}")

    if not workspace_root.is_dir():
        raise ValueError(f"Workspace root is not a directory: {workspace_root}")

    packages = []

    for package_xml_path in _iter_package_xml(workspace_root):
        try:
            packages.append(PackageXML.from_file(package_xml_path))
        except (FileNotFoundError, ValueError) as e:
            print(f"Warning: Could not parse {package_xml_path}: {e}")
    return packages


def is_workspace_package(
    package_name: str, workspace_packages: list[PackageXML]
) -> bool:
    """
    Check if a package name refers to a package in the workspace.

    Args:
        package_name: Name of the package to check
        workspace_packages: List of packages in the workspace

    Returns:
        True if package is in the workspace, False otherwise
    """
    workspace_names = {pkg.name for pkg in workspace_packages}
    return package_name in workspace_names
