# CMakeLists.txt Template

Annotated template for a dual C++/Python ROS2 package. Replace `<pkg>`, `<node>`, and `<ExtraLib>` with real values. Remove sections that don't apply (e.g. the yaml-cpp block if no YAML loading is needed).

```cmake
cmake_minimum_required(VERSION 3.8)
project(<pkg>)

# Enable strict warnings for GCC/Clang — catches common mistakes at compile time.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Core ROS2 build tools ─────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)   # CMake integration for ROS2
find_package(ament_cmake_python REQUIRED)   # Python package + script installation
find_package(rclcpp             REQUIRED)   # C++ client library
find_package(rclpy              REQUIRED)   # Python client library (needed even if only
                                            # Python scripts are installed via PROGRAMS)

# ── Optional: locate config files at runtime ──────────────────────────────────
# Include these if the C++ node reads a YAML/config file from the share directory.
find_package(ament_index_cpp REQUIRED)   # get_package_share_directory() for C++
find_package(yaml-cpp        REQUIRED)   # YAML parser

# ── Custom message packages ───────────────────────────────────────────────────
# Add any packages that define message types this node uses.
find_package(interfaces REQUIRED)   # project-local custom messages
# find_package(std_msgs  REQUIRED)
# find_package(geometry_msgs REQUIRED)

# ── Extra third-party libraries ───────────────────────────────────────────────
# find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)


# ═══════════════════════════════════════════════════════════════════════════════
# C++ executable
# ═══════════════════════════════════════════════════════════════════════════════

# Expose the include/ directory so #include "<pkg>/..." works without full paths.
include_directories(include)

add_executable(<node>_cpp src/<node>_cpp.cpp)

# Link libraries that are NOT found via ament (i.e. plain CMake targets).
# ament_index_cpp uses the modern CMake target name; yaml-cpp uses the legacy one.
target_link_libraries(<node>_cpp
  ament_index_cpp::ament_index_cpp
  yaml-cpp
  # Eigen3::Eigen
)

# Provide ament-aware dependencies (handles include paths + link libraries together).
ament_target_dependencies(<node>_cpp
  rclcpp
  interfaces
  # std_msgs
  # geometry_msgs
)

# Install the compiled binary to lib/<pkg>/ so `ros2 run <pkg> <node>_cpp` works.
install(TARGETS
  <node>_cpp
  DESTINATION lib/${PROJECT_NAME}
)


# ═══════════════════════════════════════════════════════════════════════════════
# Python package + scripts
# ═══════════════════════════════════════════════════════════════════════════════

# Registers the <pkg>/ subdirectory as an installable Python package.
# This is what makes `from <pkg> import ...` work in other nodes.
ament_python_install_package(${PROJECT_NAME})

# Install the Python node as an executable script.
# The filename (including .py) becomes the exec name in launch files.
install(PROGRAMS
  <pkg>/<node>_py.py
  DESTINATION lib/${PROJECT_NAME}
)


# ═══════════════════════════════════════════════════════════════════════════════
# Shared data files (config, YAML, etc.)
# ═══════════════════════════════════════════════════════════════════════════════

# Install config files to share/<pkg>/ so nodes can find them via the ament index
# at runtime, regardless of where the workspace is installed.
# Both the C++ (ament_index_cpp) and Python (ament_index_python) APIs can
# resolve this path automatically.
install(FILES
  config/config.yaml          # rename/add files as needed
  DESTINATION share/${PROJECT_NAME}
)


# ═══════════════════════════════════════════════════════════════════════════════
# Launch files
# ═══════════════════════════════════════════════════════════════════════════════

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


# ═══════════════════════════════════════════════════════════════════════════════
# Testing
# ═══════════════════════════════════════════════════════════════════════════════

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)   # suppress copyright linter
  set(ament_cmake_cpplint_FOUND   TRUE)   # suppress cpplint (needs git repo + licence header)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Checklist

- [ ] Removed `yaml-cpp` / `ament_index_cpp` if the C++ node doesn't load config files
- [ ] Added all message packages to `find_package` AND `ament_target_dependencies`
- [ ] Python script path in `install(PROGRAMS ...)` matches the actual file on disk
- [ ] Config file paths in `install(FILES ...)` match the actual files on disk
- [ ] `install(DIRECTORY launch ...)` is present
