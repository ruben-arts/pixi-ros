# pixi-ros

**Install your ROS workspace dependencies with Pixi**

Your ROS packages already declare their dependencies in `package.xml`.
`pixi-ros install` reads them, maps external dependencies to conda packages (primarily from [RoboStack](https://robostack.org/)), and installs them with [Pixi](https://pixi.sh).
Then use plain `pixi shell` or `pixi run build`.

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

### Install Your Workspace Dependencies

Run this from your ROS workspace:

```bash
pixi-ros install
```

There's no `init` prerequisite.
The command discovers your workspace and creates `pixi.toml` at its root, or updates an existing manifest while preserving your configuration.
It resolves dependencies from your `package.xml` files, adds build tasks, then runs `pixi install`.
Pixi must be installed first.

Choose the ROS distribution when prompted, or pass `--distro humble`.
`pixi-ros` infers the distro from existing ROS dependencies or channels; otherwise it prompts, using `ROS_DISTRO` as a default when available.
Conflicting distro settings need to be resolved rather than mixing ROS distributions.
A new workspace targets your current platform by default.
Existing workspace platforms are retained; use repeated `--platform` options to add other targets.

### Enter the Environment and Build

```bash
pixi shell
```

The shell has the ROS environment activated.
Installing dependencies doesn't build the ROS packages in your workspace.
Build those separately, with or without entering a shell:

```bash
pixi run build
pixi run test
```

When your `package.xml` files change, run `pixi-ros install` again to refresh and install their dependencies.
For day-to-day work, use Pixi directly.
Commit `pixi.toml` and the `pixi.lock` that Pixi creates so your teammates can reproduce the environment with `pixi install`.

### Only Generate the Manifest

If you want to inspect or edit the manifest before installing anything, use `init`:

```bash
pixi-ros init --distro humble
pixi install
```

`init` is a manifest-only handoff: it configures the workspace and generates a `README_PIXI.md` usage guide, but doesn't run `pixi install`.
`pixi-ros install` doesn't generate that guide.

## How It Works

### Dependency Mapping & Validation

`pixi-ros` reads all dependency types from `package.xml` files and intelligently resolves them to conda packages using a **priority-based validation system**.

#### Validation Priority Order

When resolving a ROS package dependency, `pixi-ros` checks sources in this order:

1. **Workspace packages** (local source) → Skipped, won't be added to dependencies
2. **Mapping files** → Use custom conda package mappings from the embedded mapping.
3. **ROS distribution** → Query the official ROS distro index for `ros-{distro}-{package}` packages
4. **conda-forge** (auto-detection) → Search conda-forge for packages without ros-distro prefix
5. **Custom channels** → Search any channels supplied via `--channel` (see below)
6. **NOT FOUND** → Stop installation; `init` alone leaves these as comments in `pixi.toml`

#### Package Sources

The dependency tables show where each package comes from:

- **ROS {distro}**: Official ROS distribution packages from robostack (e.g., `ros-humble-rclcpp`)
- **Mapping**: Custom mappings from YAML files (e.g., `cmake` → `cmake`, `udev` → `libusb + libudev`)
- **conda-forge**: Auto-detected packages available directly on conda-forge
- **{custom-channel-name}**: Package found in a custom channel (shown in purple, named after the channel)
- **Workspace**: Local packages in your workspace (skipped from dependencies)
- **NOT FOUND**: Packages that couldn't be resolved. Installation stops; `init` leaves them commented out in `pixi.toml`.

The mapping rules are defined in YAML files (see `src/pixi_ros/data/conda-forge.yaml`) and can be customized by placing your own mapping files in `pixi-ros/*.yaml` or `~/.pixi-ros/*.yaml`.

After dependency resolution, `pixi-ros` validates package availability in the configured channels for each target platform by connecting to `https://prefix.dev`.

### Example Output

Dependency resolution shows where each package was found.
For example:

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

The generated `pixi.toml` includes dependencies such as:

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

These constraints are written to `pixi.toml`:

```toml
[dependencies]
cmake = ">=3.12.4"
eigen = ">=3.3.0,<4.0.0"
boost = "==1.2.3"
```

## Command Reference

### `pixi-ros install`

Resolve and install external dependencies for the current ROS workspace.
Creates or updates the root `pixi.toml`, then invokes `pixi install`.
It doesn't build local ROS packages.

```bash
pixi-ros install
pixi-ros install --distro humble
pixi-ros install -d humble -p linux-64 -p osx-arm64
pixi-ros install -d humble -c https://prefix.dev/my-org
```

**Options:**
- `--distro`, `-d`: ROS distribution. Inferred from existing ROS dependencies or channels when omitted; otherwise prompts, with `ROS_DISTRO` as a possible default. Conflicting or ambiguous settings must be resolved.
- `--platform`, `-p`: Target platform, repeatable. New workspaces default to the current platform; existing platforms are retained.
- `--channel`, `-c`: Additional channel to search, repeatable. Accepts channel URLs or local paths.

Existing manifest settings are preserved while workspace dependencies are refreshed.
Run the command again after changing `package.xml` files.
Unresolved or unavailable dependencies prevent a successful installation; fix the reported dependencies rather than treating the environment as complete.
Pixi handles the environment and `pixi.lock`, so you can use `pixi install`, `pixi shell`, and `pixi run` directly afterward.

### `pixi-ros init`

Initialize or update a ROS workspace's `pixi.toml` without installing dependencies.

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
- `--channel`, `-c`: Additional channels to search for packages (optional, can be specified multiple times)
  - Accepts URLs (`https://prefix.dev/my-channel`) or local paths (`./my-channel`, `~/channels/my-channel`)
  - Channels are added to `pixi.toml` with higher priority than the default robostack and conda-forge channels
- `--dev`: Add workspace packages as source dependencies in `[dev]`, pointing to their `package.xml` files, instead of writing all resolved external dependencies into the manifest. Enables the `pixi-build` preview so Pixi can resolve these at install time.

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

### Advanced: Source Dependencies with `init --dev`

Use `init --dev` if you want Pixi to resolve workspace package source dependencies at install time:

```bash
pixi-ros init --distro humble --dev
pixi install
```

This is an alternative to the classic dependency resolution used by `pixi-ros install`.
The `install` command doesn't have a `--dev` option.

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

`init` prompts for platforms when you don't specify them.
`install` uses the current platform for new workspaces and retains platforms from an existing manifest.

```bash
$ pixi-ros init --distro humble

Available target platforms:
  1. linux-64
  2. osx-64
  3. osx-arm64
  4. win-64

Select platforms (enter numbers or names, comma or space separated): 1 3
```

## Custom Channels

If your workspace depends on packages outside the default RoboStack and conda-forge channels, pass additional channels with `--channel` / `-c`.
Both `install` and `init` accept this option.

```bash
# Single custom channel
pixi-ros init --distro kilted --channel https://prefix.dev/my-org

# Multiple custom channels
pixi-ros init --distro kilted -c https://prefix.dev/channel-a -c https://prefix.dev/channel-b

# Local channel (relative or absolute path)
pixi-ros init --distro kilted --channel ./my-local-channel
pixi-ros init --distro kilted --channel ~/channels/staging
```

### Channel priority in pixi.toml

Custom channels are written to `pixi.toml` **before** the default channels, giving them the highest resolution priority when `pixi install` runs:

```toml
[workspace]
channels = [
    "https://prefix.dev/my-org",          # custom, highest priority
    "https://prefix.dev/robostack-kilted", # default
    "https://prefix.dev/conda-forge",      # default
]
```

### Local channels

Relative and home-directory paths are accepted and preserved as-is in `pixi.toml` so pixi can resolve them relative to the workspace. Internally, `pixi-ros` resolves them to absolute `file://` URIs only for the validation step.

```bash
pixi-ros init --distro kilted --channel ./tests/data/my-channel
```

```toml
# pixi.toml: relative path kept intact
channels = [
    "./tests/data/my-channel",
    "https://prefix.dev/robostack-kilted",
    "https://prefix.dev/conda-forge",
]
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

After `pixi-ros install`, your workspace has a root manifest and Pixi lockfile:

```
workspace/
├── src/                    # Your ROS packages
│   └── my_package/
│       ├── package.xml    # ROS package manifest (source of truth)
│       └── ...
├── pixi.toml              # Generated pixi manifest
└── pixi.lock              # Locked dependencies (commit this!)
```

The optional `init` command also generates `README_PIXI.md`.
It doesn't create the lockfile; Pixi does that when you install.

## Troubleshooting

### Package Not Found

If pixi-ros marks packages as "NOT FOUND" (shown in red in the validation output):

1. **Check the ROS distro**: Verify the package exists in robostack: https://prefix.dev/channels/robostack-{distro}
2. **Check for typos**: Review your `package.xml` for spelling errors
3. **Check conda-forge**: Some packages may be available directly on conda-forge without the `ros-distro-` prefix
4. **Use a custom channel**: If the package lives in a non-default channel, pass it with `--channel`:
   ```bash
   pixi-ros init --distro humble --channel https://prefix.dev/my-org
   ```
5. **Create a mapping**: Add a custom mapping in `pixi-ros/*.yaml` if the package has a different conda name
6. **Add to workspace**: Consider including the package source in your workspace instead of depending on it
7. **Contribute to RoboStack**: If the package should be available but isn't, consider adding it to the appropriate RoboStack repository:
   - [ros-humble](https://github.com/RoboStack/ros-humble) (ROS 2)
   - [ros-jazzy](https://github.com/RoboStack/ros-jazzy) (ROS 2)
   - [ros-kilted](https://github.com/RoboStack/ros-kilted) (ROS 2)
   - [ros-lyrical](https://github.com/RoboStack/ros-lyrical) (ROS 2)
   - [ros-rolling](https://github.com/RoboStack/ros-rolling) (ROS 2)

The validation table shows exactly where each dependency was checked, making it easier to diagnose issues.

### Different Package Names

pixi-ros includes mapping files for system packages (e.g., `cmake` → `cmake`, `eigen` → `eigen`). You can override mappings by creating `pixi-ros/*.yaml` files in your workspace or `~/.pixi-ros/`.

### Platform-Specific Issues

Some packages have platform-specific mappings. pixi-ros handles this automatically, but you can test different platforms using the internal API with `platform_override`.

### Active ROS Environment Detected

If you have an existing ROS environment sourced (e.g., from `source /opt/ros/humble/setup.bash` in your `~/.bashrc`), `pixi-ros init` will warn you:

```
⚠️  Important: Active ROS environment detected!
   Pixi manages the environment automatically. Remove any source /opt/ros/...
   or source install/setup.bash from your shell config (e.g. ~/.bashrc).
```

This warning appears when any of these environment variables are detected:
- `ROS_DISTRO` - Indicates a sourced ROS distribution
- `ROS_VERSION` - Set by ROS 1 or ROS 2
- `AMENT_PREFIX_PATH` - ROS 2 specific variable
- `ROS_PACKAGE_PATH` - ROS 1 specific variable

**Why this matters:** Having a manually sourced ROS environment can conflict with pixi's managed environment, leading to version mismatches and unexpected behavior. Pixi automatically sources the correct setup scripts when you use `pixi shell` or `pixi run`, so manual sourcing is unnecessary.

**How to fix:**
1. Open your shell configuration file (e.g., `~/.bashrc`, `~/.zshrc`)
2. Remove or comment out any lines like:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros_ws/install/setup.bash
   ```
3. Restart your shell or run `source ~/.bashrc` (or equivalent)

## Contributing

Contributions welcome! Feel free to open issues or PRs on GitHub.

## Learn More

- Pixi: https://pixi.sh
- RoboStack: https://robostack.org/
- conda-forge: [https://](https://conda-forge.org/)
- Conda: https://docs.conda.io/
- ROS 2: https://docs.ros.org/

## Disclaimer

This tool is built with heavy use of AI assistance and is under active development.
Please report issues or contribute on GitHub!

I (Ruben) hope `pixi-ros` can die ASAP, as all of the workflows this tool provides should ideally be native to Pixi itself.
Until then, use it to install your ROS dependencies and let Pixi take it from there!
