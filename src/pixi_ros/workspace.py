"""Workspace discovery and management for ROS packages."""

from pathlib import Path

from pixi_ros.package_xml import PackageXML


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
    Find the workspace root by looking for a directory structure with package.xml files.

    A workspace root is typically identified by having a 'src' directory containing
    package.xml files, or by being a directory that contains package.xml files.

    Args:
        start_path: Starting directory for search (defaults to cwd)

    Returns:
        Path to workspace root if found, None otherwise
    """
    if start_path is None:
        start_path = Path.cwd()

    current = start_path.resolve()

    # First, check if we're inside a package - search upward for package.xml
    package_xml = find_package_xml(current)
    if package_xml:
        # We found a package.xml, so go up one level to see if there's a src/ dir
        package_dir = package_xml.parent
        potential_src = package_dir.parent

        # Check if parent directory is named 'src'
        if potential_src.name == "src":
            # The workspace root is the parent of 'src'
            return potential_src.parent

        # Otherwise, check if current directory has a src/ folder with packages
        if (current / "src").exists():
            if any((current / "src").glob("*/package.xml")):
                return current

    # Search upward for a directory with 'src' containing package.xml files
    while True:
        src_dir = current / "src"
        if src_dir.exists() and src_dir.is_dir():
            # Check if src contains any package.xml files
            if any(src_dir.glob("*/package.xml")) or any(src_dir.glob("package.xml")):
                return current

        parent = current.parent
        if parent == current:  # Reached root
            break
        current = parent

    return None


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
    skip_dirs = {"build", "install", "log", ".pixi"}

    # Recursively find all package.xml files
    for package_xml_path in workspace_root.rglob("package.xml"):
        # Get relative path parts from workspace root
        relative_parts = package_xml_path.relative_to(workspace_root).parts

        # Skip if any parent directory is hidden (starts with .) or in skip list
        if any(part.startswith(".") or part in skip_dirs for part in relative_parts):
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
