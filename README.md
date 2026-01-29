# Pixi-ros is a Pixi extension to initialize ros packages.

```bash
# Initialize a `pixi.toml` for the ros workspace you are in.
pixi ros init --distro <ros_distro> 

# Create a new ROS package with the given name.
# Both Python and C++ are supported.
pixi ros pkg create <package_name> --python
pixi ros pkg create <package_name> --cpp

# Add a dependency to the current package, and the pixi.toml.
pixi ros add <dependency_name>
```

## Installation

To install Pixi-ros, you can use `pixi global` command:
```bash
pixi global install pixi-ros
```

## Philosophy

Pixi-ros aims to simplify the process of initializing ROS packages by providing a set of commands that automate common tasks. It is designed to be easy to use and integrate seamlessly with existing Pixi workflows.

You should be able to initialize a ROS package with just a few commands, without needing to manually set up the package structure or configuration files.

This extension has the ability to read package.xml files and translate it into a Pixi manifest, allowing for easy migration of existing ROS packages to Pixi.

We hope this package becomes obsolete in the future. But for the time being, it serves as a gateway drug to help ROS developers transition to Pixi.

`pixi ros init` should be able to be run multiple times, and should update the existing `pixi.toml` file with any new information.

## Out of scope

- The `ros2` cli as a tool is not a goal of this extension. While some commands may overlap with `ros2` commands, the focus of Pixi-ros is to provide a Pixi-centric experience for ROS developers.
- The `colcon` is not replaced by this extension and might even use or provide the user with it. 
- This extension does not aim to provide a full-featured ROS development environment. Instead, it focuses on simplifying the 
