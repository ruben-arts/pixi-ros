"""Parser for ROS package.xml files."""

from dataclasses import dataclass, field
from pathlib import Path

from lxml import etree


@dataclass
class PackageXML:
    """Represents a parsed ROS package.xml file."""

    name: str
    version: str
    description: str
    maintainer: str
    maintainer_email: str
    license: str
    format_version: int
    build_type: str | None = None
    path: Path | None = None  # Path to the package.xml file

    # Different dependency categories
    buildtool_depends: list[str] = field(default_factory=list)
    build_depends: list[str] = field(default_factory=list)
    build_export_depends: list[str] = field(default_factory=list)
    exec_depends: list[str] = field(default_factory=list)
    test_depends: list[str] = field(default_factory=list)

    # Format 2 compatibility (run_depend)
    run_depends: list[str] = field(default_factory=list)

    # Generic depends (shorthand for build, export, and exec)
    depends: list[str] = field(default_factory=list)

    # Version constraints for dependencies
    # Maps package name to version constraint string
    # (e.g., ">=3.12.4", ">=1.8.0,<2.0.0")
    dependency_versions: dict[str, str] = field(default_factory=dict)

    @classmethod
    def from_file(cls, path: Path) -> "PackageXML":
        """
        Parse a package.xml file.

        Args:
            path: Path to the package.xml file

        Returns:
            PackageXML object with parsed data

        Raises:
            FileNotFoundError: If the file doesn't exist
            ValueError: If the XML is malformed or missing required fields
        """
        if not path.exists():
            raise FileNotFoundError(f"package.xml not found at {path}")

        try:
            tree = etree.parse(str(path))
            root = tree.getroot()
        except etree.XMLSyntaxError as e:
            raise ValueError(f"Invalid XML in {path}: {e}") from e

        # Get format version (defaults to 1 if not specified)
        format_version = int(root.get("format", "1"))

        # Extract required fields
        name_elem = root.find("name")
        version_elem = root.find("version")
        description_elem = root.find("description")
        maintainer_elem = root.find("maintainer")
        license_elem = root.find("license")

        if name_elem is None or name_elem.text is None:
            raise ValueError(f"Missing required 'name' field in {path}")
        if version_elem is None or version_elem.text is None:
            raise ValueError(f"Missing required 'version' field in {path}")
        if description_elem is None or description_elem.text is None:
            raise ValueError(f"Missing required 'description' field in {path}")
        if maintainer_elem is None or maintainer_elem.text is None:
            raise ValueError(f"Missing required 'maintainer' field in {path}")
        if license_elem is None or license_elem.text is None:
            raise ValueError(f"Missing required 'license' field in {path}")

        # Extract maintainer email
        maintainer_email = maintainer_elem.get("email", "")

        # Extract build type from export section
        build_type = None
        export_elem = root.find("export")
        if export_elem is not None:
            build_type_elem = export_elem.find("build_type")
            if build_type_elem is not None and build_type_elem.text:
                build_type = build_type_elem.text

        # Extract dependencies
        def parse_version_constraint(elem) -> str | None:
            """
            Parse version constraint attributes from a dependency element.

            Converts ROS package.xml version attributes to conda/pixi constraint syntax:
            - version_lt="X" → <X
            - version_lte="X" → <=X
            - version_eq="X" → ==X
            - version_gte="X" → >=X
            - version_gt="X" → >X

            Multiple constraints are combined with commas.
            """
            constraints = []

            version_attrs = [
                ("version_lt", "<"),
                ("version_lte", "<="),
                ("version_eq", "=="),
                ("version_gte", ">="),
                ("version_gt", ">"),
            ]

            for attr, op in version_attrs:
                value = elem.get(attr)
                if value:
                    constraints.append(f"{op}{value}")

            return ",".join(constraints) if constraints else None

        def get_deps(tag: str) -> list[str]:
            """Extract all dependencies with the given tag."""
            deps = []
            for elem in root.findall(tag):
                if elem.text:
                    deps.append(elem.text.strip())
            return deps

        def get_deps_with_versions(tag: str, version_map: dict[str, str]) -> list[str]:
            """Extract dependencies and populate version constraints."""
            deps = []
            for elem in root.findall(tag):
                if elem.text:
                    pkg_name = elem.text.strip()
                    deps.append(pkg_name)

                    # Parse version constraint if present
                    constraint = parse_version_constraint(elem)
                    if constraint:
                        # If package already has a constraint, combine them
                        if pkg_name in version_map:
                            version_map[pkg_name] = (
                                f"{version_map[pkg_name]},{constraint}"
                            )
                        else:
                            version_map[pkg_name] = constraint
            return deps

        # Parse all dependency types and collect version constraints
        dependency_versions: dict[str, str] = {}
        buildtool_depends = get_deps_with_versions(
            "buildtool_depend", dependency_versions
        )
        build_depends = get_deps_with_versions("build_depend", dependency_versions)
        build_export_depends = get_deps_with_versions(
            "build_export_depend", dependency_versions
        )
        exec_depends = get_deps_with_versions("exec_depend", dependency_versions)
        test_depends = get_deps_with_versions("test_depend", dependency_versions)
        depends = get_deps_with_versions("depend", dependency_versions)

        # Format 2 compatibility
        run_depends = get_deps_with_versions("run_depend", dependency_versions)

        return cls(
            name=name_elem.text.strip(),
            version=version_elem.text.strip(),
            description=description_elem.text.strip(),
            maintainer=maintainer_elem.text.strip(),
            maintainer_email=maintainer_email,
            license=license_elem.text.strip(),
            format_version=format_version,
            build_type=build_type,
            path=path,
            buildtool_depends=buildtool_depends,
            build_depends=build_depends,
            build_export_depends=build_export_depends,
            exec_depends=exec_depends,
            test_depends=test_depends,
            run_depends=run_depends,
            depends=depends,
            dependency_versions=dependency_versions,
        )

    def get_all_build_dependencies(self) -> list[str]:
        """
        Get all dependencies needed at build time.

        Returns:
            Combined list of buildtool, build, and generic depends
        """
        deps = set()
        deps.update(self.buildtool_depends)
        deps.update(self.build_depends)
        deps.update(self.depends)
        return sorted(deps)

    def get_all_runtime_dependencies(self) -> list[str]:
        """
        Get all dependencies needed at runtime.

        Handles both format 2 (run_depend) and format 3 (exec_depend).

        Returns:
            Combined list of exec, run, and generic depends
        """
        deps = set()
        deps.update(self.exec_depends)
        deps.update(self.run_depends)  # Format 2 compatibility
        deps.update(self.depends)
        return sorted(deps)

    def get_all_test_dependencies(self) -> list[str]:
        """
        Get all dependencies needed for testing.

        Returns:
            List of test depends
        """
        return sorted(set(self.test_depends))

    def get_all_dependencies(self) -> list[str]:
        """
        Get all unique dependencies across all categories.

        Returns:
            Combined, deduplicated list of all dependencies
        """
        deps = set()
        deps.update(self.buildtool_depends)
        deps.update(self.build_depends)
        deps.update(self.build_export_depends)
        deps.update(self.exec_depends)
        deps.update(self.run_depends)
        deps.update(self.depends)
        deps.update(self.test_depends)
        return sorted(deps)
