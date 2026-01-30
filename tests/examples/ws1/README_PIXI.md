# Pixi-ROS Workspace

This ROS jazzy workspace is configured to use [Pixi](https://pixi.sh) for dependency management.

## What is Pixi?

Pixi is a modern package manager that uses conda packages. It provides:
- Fast, reproducible dependency resolution
- Automatic environment management
- Reproducible with a lockfile
- Cross-platform support (Linux, macOS, Windows)

## Getting Started

### 1. Install Dependencies

```bash
pixi install
```

This installs all dependencies specified in `pixi.toml` from the configured channels.

### 2. Build the Workspace

```bash
pixi run build
```

This runs `colcon build` to compile your ROS packages.

### 3. Run Tests

```bash
pixi run test
```

This runs `colcon test` to execute your test suite.

### 4. Clean Build Artifacts

```bash
pixi run clean
```

Removes `build/`, `install/`, and `log/` directories.

### 5. Add additional dependencies

```bash
pixi add <package-name>
# or for packages you would install with pip/uv/poetry:
pixi add --pypi <package-name>
```

This adds new dependencies to `pixi.toml` and installs them.

## Environment Activation

After the first build, pixi will automatically source the ROS setup script (`install/setup.bash`)
when you enter the pixi environment. This means you don't need to manually source it!

To activate the environment, run:

```bash
pixi shell
```

This starts a new shell with the ROS environment activated.

The environment will also be automatically activated when you run commands with `pixi run <command>`.

## Adding Dependencies

### Add a Conda Package

```bash
pixi add <package-name>
```

### Add ROS Dependencies

When you add dependencies to your `package.xml` files, run:

```bash
pixi ros init --distro jazzy
```

This will update `pixi.toml` with the new dependencies.

## Unavailable Packages

If you see commented-out packages in `pixi.toml` with `# NOT FOUND`, these packages
were not found in the configured channels. You may need to:
- Check if the package name is correct
- Add additional channels with `pixi project channel add <channel-url>`
- Install the package through pip: `pixi add --pypi <package-name>`
- Add it to [conda-forge](https://github.com/conda-forge/staged-recipes) or [RoboStack](https://robostack.github.io/Contributing.html)

## Common Issues

### Build Fails

If `pixi run build` fails:
1. Make sure all dependencies are installed: `pixi install`
2. Clean and rebuild: `pixi run clean && pixi run build`
3. Validate the build task is correct for your workspace.

### Environment Issues

If `ros2` commands aren't found:
1. Run commands through pixi: `pixi run <command>`
2. Or use a pixi shell: `pixi shell`

## Learn More

- **Pixi Documentation**: https://pixi.sh
- **RoboStack Documentation**: https://robostack.github.io/
- **ROS jazzy Documentation**: https://docs.ros.org/en/jazzy/
- **pixi-ros GitHub**: https://github.com/prefix-dev/pixi-ros

## Channels

This workspace uses the following channels:
- `https://prefix.dev/robostack-jazzy` - ROS packages
- `https://prefix.dev/conda-forge` - System dependencies
