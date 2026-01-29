# Mapping files

This file contains mappings from ROS package names to their conda package equivalents.
These are split by channel. e.g. conda-forge.

## Format

The file is based on the rosdep format, with the following structure:

- Keys are ROS package names, as presented in the package.xml (e.g. `udev`, `uncrustify`, `uuid`). 
- Values are dictionaries mapping channels to lists of conda package names.

Example:

```yaml
udev:
  pixi:
    linux: [libusb, libudev]
    osx: [libusb]
    win64: [libusb]
uncrustify:
  pixi: [uncrustify]
unzip:
  pixi: [unzip]
uuid:
  pixi:
    linux: [libuuid]
    osx: []
    win64: []
virtualenv:
  pixi: [virtualenv]
```

## Customization

You can override these mappings by creating your own `pixi-ros/channel.yaml` file in:

1. **Workspace directory** (highest priority): `./pixi-ros/channel.yaml`
2. **User config directory**: `~/.pixi-ros/channel.yaml`
3. **Built-in defaults** (these file): Used if no custom file is found

If you use the same channel name it will override the file completely.
The channel names are not used for specific package lookups, only to find the correct mapping file.
e.g. if you name the file `foobar.yaml` it will still be able to fetch the packages from other channels.


## Fallback Behavior

If a package is not found in channel.yaml, pixi-ros will automatically:
1. Convert underscores to dashes
2. Prepend `ros-{distro}-`

For example, `my_custom_package` becomes `ros-humble-my-custom-package`.

## Local packages
If a package is found in the `src` folder and also defined in the mapping, it will use the local package.
