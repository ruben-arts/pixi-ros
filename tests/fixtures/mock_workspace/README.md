# Mock ROS Workspace

This is a mock ROS workspace for testing pixi-ros functionality.

## Structure

```
mock_workspace/
└── src/
    ├── my_python_pkg/      - Python package (format 3)
    ├── my_cpp_pkg/         - C++ package (format 3) with separated build/exec deps
    ├── my_mixed_pkg/       - Complex package with various dependency types
    └── legacy_pkg/         - Legacy package using format 2
```

## Package Details

### my_python_pkg
- **Type**: Python (ament_python)
- **Format**: 3
- **Dependencies**: rclpy, std_msgs, geometry_msgs
- **Test deps**: pytest, ament_lint

### my_cpp_pkg
- **Type**: C++ (ament_cmake)
- **Format**: 3
- **Dependencies**: rclcpp, std_msgs, sensor_msgs
- **Test deps**: ament_cmake_gtest

### my_mixed_pkg
- **Type**: Mixed (ament_cmake with Python)
- **Format**: 3
- **Dependencies**: Multiple types (build, exec, build_export)
- **Workspace deps**: Depends on my_cpp_pkg and my_python_pkg
- **Notable**: Uses build_export_depend for libraries

### legacy_pkg
- **Type**: Legacy catkin package
- **Format**: 2
- **Dependencies**: Uses run_depend instead of exec_depend
- **Notable**: ROS 1 style dependencies
