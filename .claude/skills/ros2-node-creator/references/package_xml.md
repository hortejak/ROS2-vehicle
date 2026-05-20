# package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name><pkg></name>
  <version>0.0.0</version>
  <description>One-line description of what this node does.</description>
  <maintainer email="hortensky.jakub@gmail.com">hortejak</maintainer>
  <license>TODO: License declaration</license>

  <!-- Build tools: ament_cmake drives the CMake build;
       ament_cmake_python adds Python package/script installation support. -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <!-- Runtime + build dependencies.
       Use <depend> (= build + exec) for libraries the node links against or imports.
       Use <build_depend> only for things needed at compile time but not at runtime.
       Use <exec_depend> only for things needed at runtime but not at compile time. -->
  <depend>rclcpp</depend>
  <depend>rclpy</depend>

  <!-- Custom messages from this workspace -->
  <depend>interfaces</depend>

  <!-- Uncomment as needed:
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>yaml-cpp</depend>
  -->

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <!-- This tells colcon to use ament_cmake as the build system.
         Without this, colcon may try to use plain CMake and skip ament features. -->
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Notes

- `yaml-cpp` is a system library; it doesn't need a `<depend>` entry in package.xml because it's found via `find_package(yaml-cpp)` in CMake, not through the ament dependency resolver. Adding it here is harmless but not necessary unless you want `rosdep` to install it automatically.
- Always keep `rclpy` even if the Python node is thin — removing it can cause import errors in some ROS2 distributions.
- The `<build_type>ament_cmake</build_type>` export is mandatory for dual C++/Python packages; without it colcon may treat the package as `ament_python` and skip C++ compilation.
