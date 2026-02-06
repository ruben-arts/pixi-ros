# pixi-ros

**Bridge your ROS workspace to the modern conda/Pixi ecosystem**

pixi-ros helps ROS developers transition from `rosdep` to [Pixi](https://pixi.sh) for package management.
It automatically reads your ROS workspace's `package.xml` files and generates a `pixi.toml` manifest with all dependencies resolved from conda channels (primarily [robostack](https://robostack.org/)).

## Quick Start

### Installation

Install pixi first if you haven't already:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Or follow instructions at https://pixi.sh/latest/installation/

Install pixi-ros globally using pixi:

```bash
pixi global install pixi-ros
```

### Initialize Your ROS Workspace

Navigate to your ROS workspace and run:

```bash
pixi-ros init --distro humble
```

This will:
1. Discover all ROS packages in your workspace (by finding `package.xml` files)
2. Read dependencies from each `package.xml`
3. **Validate and resolve** each dependency using the priority system:
   - Skip workspace packages (built locally)
   - Use custom mappings from YAML files
   - Query ROS distro index for ROS packages
   - Auto-detect packages on conda-forge
   - Flag packages that can't be found
4. Generate/update `pixi.toml` with proper channels and dependencies
5. Check package availability in conda channels for each platform
6. Create helpful build/test/clean tasks
7. Display detailed validation results with source information

### Install and Build

After initialization, use standard pixi commands:

```bash
# Install all dependencies
pixi install

# Build your workspace
pixi run build

# Run tests
pixi run test

# Activate environment for direct ROS commands
pixi shell
```

## How It Works

### Dependency Mapping & Validation

`pixi-ros` reads all dependency types from `package.xml` files and intelligently resolves them to conda packages using a **priority-based validation system**.

#### Validation Priority Order

When resolving a ROS package dependency, `pixi-ros` checks sources in this order:

1. **Workspace packages** (local source) → Skipped, won't be added to dependencies
2. **Mapping files** → Use custom conda package mappings from the embedded mapping.
3. **ROS distribution** → Query the official ROS distro index for `ros-{distro}-{package}` packages
4. **conda-forge** (auto-detection) → Search conda-forge for packages without ros-distro prefix
5. **NOT FOUND** → Mark as unavailable and comment out in `pixi.toml`

#### Package Sources

The dependency tables show where each package comes from:

- **ROS {distro}**: Official ROS distribution packages from robostack (e.g., `ros-humble-rclcpp`)
- **Mapping**: Custom mappings from YAML files (e.g., `cmake` → `cmake`, `udev` → `libusb + libudev`)
- **conda-forge**: Auto-detected packages available directly on conda-forge
- **Workspace**: Local packages in your workspace (skipped from dependencies)
- **NOT FOUND**: Packages that couldn't be resolved (commented out in `pixi.toml`)

The mapping rules are defined in YAML files (see `src/pixi_ros/data/conda-forge.yaml`) and can be customized by placing your own mapping files in `pixi-ros/*.yaml` or `~/.pixi-ros/*.yaml`.

After dependency resolution, `pixi-ros` validates package availability in the configured channels for each target platform by connecting to `https://prefix.dev`.

### Example Output

When you run `pixi-ros init --distro humble`, you'll see validation results:

```
Found 2 package(s): my_package, other_package
Initializing ROS humble distribution validator...

╭─────────────────── Package: my_package ───────────────────╮
│ ROS Dependency │ Type    │ Conda Packages          │ Source       │
├────────────────┼─────────┼─────────────────────────┼──────────────┤
│ rclcpp         │ Build   │ ros-humble-rclcpp       │ ROS humble   │
│ std_msgs       │ Runtime │ ros-humble-std-msgs     │ ROS humble   │
│ cmake          │ Build   │ cmake                   │ Mapping      │
│ eigen          │ Build   │ eigen                   │ conda-forge  │
╰────────────────┴─────────┴─────────────────────────┴──────────────╯

Validation Summary:
  ✓ 2 workspace packages (skipped)
  ✓ 1 packages from mappings
  ✓ 5 packages from ROS humble distro
  ✓ 1 packages from conda-forge (auto-detected)

  Total external dependencies: 7
```

Given a `package.xml` with:

```xml
<depend>rclcpp</depend>
<build_depend>ament_cmake</build_depend>
<build_depend>cmake</build_depend>
<exec_depend>std_msgs</exec_depend>
```

`pixi-ros init --distro humble` generates a `pixi.toml` with:

```toml
[dependencies]
# Base ROS dependencies
ros-humble-ros-base = "*"
pkg-config = "*"
compilers = "*"
make = "*"
ninja = "*"

# Build tools
colcon-common-extensions = "*"

# Workspace dependencies
cmake = "*"                      # From mapping
ros-humble-ament-cmake = "*"    # From ROS humble
ros-humble-rclcpp = "*"         # From ROS humble
ros-humble-std-msgs = "*"       # From ROS humble
```

### Version Constraints

`pixi-ros` supports version constraints from `package.xml` files and automatically applies them to the generated `pixi.toml`.

#### Supported Version Attributes

You can specify version requirements in your `package.xml` using standard ROS version attributes:

| package.xml attribute | pixi.toml constraint | Description |
|----------------------|----------------------|-------------|
| `version_eq="X.Y.Z"` | `==X.Y.Z` | Exactly version X.Y.Z |
| `version_gte="X.Y.Z"` | `>=X.Y.Z` | Version X.Y.Z or newer |
| `version_gt="X.Y.Z"` | `>X.Y.Z` | Newer than version X.Y.Z |
| `version_lte="X.Y.Z"` | `<=X.Y.Z` | Version X.Y.Z or older |
| `version_lt="X.Y.Z"` | `<X.Y.Z` | Older than version X.Y.Z |

Multiple constraints can be combined on the same dependency and will be joined with commas in the output.

Given a `package.xml` with version constraints:

```xml
<depend version_gte="3.12.4">cmake</depend>
<build_depend version_gte="3.3.0" version_lt="4.0.0">eigen</build_depend>
<exec_depend version_eq="1.2.3">boost</exec_depend>
```

`pixi-ros init` generates:

```toml
[dependencies]
cmake = ">=3.12.4"
eigen = ">=3.3.0,<4.0.0"
boost = "==1.2.3"
```

## Command Reference

### `pixi-ros init`

Initialize or update a ROS workspace's `pixi.toml`.

```bash
pixi-ros init --distro <ros_distro>
pixi-ros init --distro humble --platform linux-64 --platform osx-arm64
pixi-ros init
```

**Options:**
- `--distro`, `-d`: ROS distribution (optional, will prompt if not provided)
- `--platform`, `-p`: Target platforms (optional, can be specified multiple times, will prompt if not provided)
  - Available: `linux-64`, `osx-64`, `osx-arm64`, `win-64`
  - Platforms come from the mapping files and determine which dependencies are available

**What it does:**
- Scans workspace for `package.xml` files
- Reads all dependency types (build, exec, test) and version constraints
- **Validates dependencies** using the priority-based system (workspace → mapping → ROS distro → conda-forge)
- Maps ROS dependencies to conda packages for each platform
- Applies version constraints from package.xml to pixi.toml dependencies
- Configures robostack channels
- Checks package availability per platform
- Creates build tasks using colcon
- Generates helpful `README_PIXI.md`
- Sets up platform-specific dependencies in `pixi.toml`
- **Displays validation results** showing where each dependency was found

**Running multiple times:**
The command is idempotent - you can run it multiple times to update dependencies as your workspace changes.

## Multi-Platform Support

`pixi-ros` supports generating cross-platform configurations. When you specify multiple platforms, it:

1. **Analyzes dependencies per platform**: Some packages have platform-specific mappings (e.g., OpenGL requirements differ between Linux and macOS)

2. **Organizes dependencies intelligently**:
   - **Common dependencies** (available on all platforms) → `[dependencies]`
   - **Unix dependencies** (available on Linux and macOS, but not Windows) → `[target.unix.dependencies]`
   - **Platform-specific dependencies** → `[target.linux.dependencies]`, `[target.osx.dependencies]`, etc.

3. **Sets up correct platform list**: The `[workspace]` section gets the appropriate pixi platform names

### Platform Naming

pixi-ros uses standard pixi platform names:
- `linux-64` - Linux x86_64
- `osx-64` - macOS Intel
- `osx-arm64` - macOS Apple Silicon (M1/M2/M3)
- `win-64` - Windows x86_64

Internally, mapping files use a simplified format (`linux`, `osx`, `win64`), but this is transparent to users. When you specify `osx-64` and `osx-arm64`, they both use the same `osx` mapping rules since package availability is typically the same for both architectures.

### Example: Multi-Platform Setup

```bash
pixi-ros init --distro humble --platform linux-64 --platform osx-arm64
```

Generates:

```toml
[workspace]
name = "my_workspace"
channels = [
    "https://prefix.dev/robostack-humble",
    "https://prefix.dev/conda-forge",
]
platforms = ["linux-64", "osx-arm64"]

[dependencies]
# Common dependencies (available on all platforms)
ros-humble-rclcpp = "*"
ros-humble-std-msgs = "*"

[target.unix.dependencies]
# Unix-specific dependencies (Linux and macOS)
xorg-libx11 = "*"
xorg-libxext = "*"

[target.linux.dependencies]
# Linux-specific dependencies
libgl-devel = "*"
libopengl-devel = "*"
```

### Interactive Platform Selection

If you don't specify platforms, you'll be prompted:

```bash
$ pixi-ros init --distro humble

Available target platforms:
  1. linux-64
  2. osx-64
  3. osx-arm64
  4. win-64

Select platforms (enter numbers or names, comma or space separated): 1 3
```

## Philosophy

`pixi-ros` aims to be a quick **gateway drug**. It:

- Respects existing ROS conventions (package.xml as source of truth)
- Uses standard ROS build tools (colcon)
- Focuses only on dependency management and environment setup
- Doesn't replace `ros2` CLI or other ROS tooling
- Should eventually become unnecessary as the ecosystem matures

Think of it as a "gateway" to help ROS developers benefit from modern package management while keeping familiar workflows.

## Project Structure

After initialization, your workspace will have:

```
workspace/
├── src/                    # Your ROS packages
│   └── my_package/
│       ├── package.xml    # ROS package manifest (source of truth)
│       └── ...
├── pixi.toml              # Generated pixi manifest
├── pixi.lock              # Locked dependencies (commit this!)
└── README_PIXI.md         # Generated usage guide
```

## Troubleshooting

### Package Not Found

If pixi-ros marks packages as "NOT FOUND" (shown in red in the validation output):

1. **Check the ROS distro**: Verify the package exists in robostack: https://prefix.dev/channels/robostack-{distro}
2. **Check for typos**: Review your `package.xml` for spelling errors
3. **Check conda-forge**: Some packages may be available directly on conda-forge without the `ros-distro-` prefix
4. **Create a mapping**: Add a custom mapping in `pixi-ros/*.yaml` if the package has a different conda name
5. **Add to workspace**: Consider including the package source in your workspace instead of depending on it
6. **Contribute to RoboStack**: If the package should be available but isn't, consider adding it to the appropriate RoboStack repository:
   - [ros-humble](https://github.com/RoboStack/ros-humble) (ROS 2)
   - [ros-jazzy](https://github.com/RoboStack/ros-jazzy) (ROS 2)
   - [ros-kilted](https://github.com/RoboStack/ros-kilted) (ROS 2)

The validation table shows exactly where each dependency was checked, making it easier to diagnose issues.

### Different Package Names

pixi-ros includes mapping files for system packages (e.g., `cmake` → `cmake`, `eigen` → `eigen`). You can override mappings by creating `pixi-ros/*.yaml` files in your workspace or `~/.pixi-ros/`.

### Platform-Specific Issues

Some packages have platform-specific mappings. pixi-ros handles this automatically, but you can test different platforms using the internal API with `platform_override`.

## Contributing

Contributions welcome! Feel free to open issues or PRs on GitHub.

## Learn More

- Pixi: https://pixi.sh
- RoboStack: https://robostack.org/
- conda-forge: [https://](https://conda-forge.org/)
- Conda: https://docs.conda.io/
- ROS 2: https://docs.ros.org/

## Disclaimer

This tool is build with heavy use of AI assistance and is under active development. Please report issues or contribute on GitHub!

I (Ruben) hope `pixi-ros` can die ASAP, as all of the workflows this tool provides should ideally be native to Pixi itself. But until then, I hope this initialization tool helps you get started!
