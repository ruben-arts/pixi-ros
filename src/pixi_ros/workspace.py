"""Workspace discovery and management for ROS packages."""

from pathlib import Path

import pathspec

from pixi_ros.package_xml import PackageXML


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
    """
    Find the workspace root by looking for a directory with ROS package.xml files.

    Searches recursively for package.xml files, excluding hidden directories and
    build artifacts. Returns the directory containing packages, or the parent of
    a 'src' directory if packages are organized in that structure.

    Args:
        start_path: Starting directory for search (defaults to cwd)

    Returns:
        Path to workspace root if found, None otherwise
    """
    if start_path is None:
        start_path = Path.cwd()

    current = start_path.resolve()
    skip_dirs = {"build", "install", "log", ".pixi"}

    # Helper to check if a directory has package.xml files
    def has_packages(path: Path) -> bool:
        """Check if path contains any package.xml files (recursively)."""
        # Load gitignore patterns if available
        gitignore_spec = load_gitignore_spec(path)

        for package_xml in path.rglob("package.xml"):
            # Get relative path from workspace root
            relative_path = package_xml.relative_to(path)
            relative_parts = relative_path.parts

            # Skip if any parent is hidden or in skip list
            if any(
                part.startswith(".") or part in skip_dirs for part in relative_parts
            ):
                continue

            # Skip if matched by gitignore patterns
            if gitignore_spec and gitignore_spec.match_file(str(relative_path)):
                continue

            return True
        return False

    # First, check if we're inside a package - search upward for package.xml
    package_xml = find_package_xml(current)
    if package_xml:
        # We found a package.xml, so determine the workspace root
        package_dir = package_xml.parent
        potential_src = package_dir.parent

        # Check if parent directory is named 'src'
        if potential_src.name == "src":
            # The workspace root is the parent of 'src'
            return potential_src.parent

        # Otherwise, return the parent of the package directory
        return package_dir.parent

    # Check if current directory has any packages
    if has_packages(current):
        # If the directory is named 'src', return its parent as the workspace root
        if current.name == "src":
            return current.parent
        return current

    # Search upward for a directory containing packages
    while True:
        if has_packages(current):
            # If the directory is named 'src', return its parent as the workspace root
            if current.name == "src":
                return current.parent
            return current

        parent = current.parent
        if parent == current:  # Reached root
            break
        current = parent

    # If no packages found anywhere, return the starting directory
    # This allows initializing a new workspace in an empty directory
    return start_path.resolve()


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

    # Directories to skip during recursive search
    skip_dirs = {"build", "install", "log", ".pixi", "tests", "test"}

    # Load gitignore patterns if available
    gitignore_spec = load_gitignore_spec(workspace_root)

    # Recursively find all package.xml files
    for package_xml_path in workspace_root.rglob("package.xml"):
        # Get relative path from workspace root
        relative_path = package_xml_path.relative_to(workspace_root)
        relative_parts = relative_path.parts

        # Skip if any parent directory is hidden (starts with .) or in skip list
        if any(part.startswith(".") or part in skip_dirs for part in relative_parts):
            continue

        # Skip if matched by gitignore patterns
        if gitignore_spec and gitignore_spec.match_file(str(relative_path)):
            continue

        try:
            package = PackageXML.from_file(package_xml_path)
            packages.append(package)
        except (FileNotFoundError, ValueError) as e:
            # Log warning but continue
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
