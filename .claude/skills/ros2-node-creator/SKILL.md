---
name: ros2-node-creator
description: Scaffold a new ROS2 node package following this project's dual C++/Python pattern. Use this skill whenever the user asks to create a new ROS2 node, package, publisher, or subscriber — even if they just say "add a node for X" or "I need a package that does Y". Covers the full directory structure, CMakeLists.txt, package.xml, C++ and Python implementations, launch files, config/YAML file inclusion, and a README.md. Always trigger for any "create a node", "new package", "add a publisher", "add a subscriber", or "scaffold a ROS2 node" request in this workspace.
---

# ROS2 Node Creator

This skill scaffolds new ROS2 packages following the pattern in `src/vehicle/vcon/`. That node is the canonical example — it hosts both a C++ and a Python implementation, loads a YAML config file from the installed package share directory, and is built with `ament_cmake` + `ament_cmake_python`. Every new node in this workspace should follow the same layout.

## Step 1 — Gather Requirements

Before writing a single file, collect:

1. **Package name** — snake_case (e.g. `lidar_preprocessor`)
2. **Location** within `src/` — e.g. `sensors/`, `vehicle/`, `planning/`, `misc/`
3. **Node purpose** — one sentence: what it subscribes to, what it publishes, what it does
4. **Messages** — does it use types from `interfaces`? Which ones?
5. **Config file** — does it need a YAML file at runtime (like VCON.yaml)? What parameters?
6. **Extra libraries** — e.g. `yaml-cpp`, `Eigen`, `OpenCV`

If any of these are missing, ask before generating files. Getting this right up front avoids regenerating half the scaffolding.

## Step 2 — Directory Structure

Create this layout verbatim (substitute `<pkg>` and `<node>` with the actual names):

```
src/<location>/<pkg>/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── <pkg>/
│       └── <node>_cpp.hpp        ← C++ header
├── src/
│   └── <node>_cpp.cpp            ← C++ source
├── <pkg>/                        ← Python package directory (same name as pkg)
│   ├── __init__.py
│   └── <node>_py.py
├── launch/
│   ├── <node>_cpp.xml
│   └── <node>_py.xml
└── config/                       ← only if a config file is needed
    └── config.yaml
```

The Python directory shares the ROS package name so `ament_python_install_package` registers it as a proper importable Python module — other nodes can then `from <pkg> import ...`.

## Step 3 — Generate the Files

Read the four reference files before writing any code. They contain full, copy-ready templates:

- `references/cmakelists.md` — annotated CMakeLists.txt
- `references/package_xml.md` — package.xml template
- `references/cpp_node.md` — C++ header and source templates
- `references/python_node.md` — Python node template

Adapt each template to the specific package: replace placeholder names, add/remove dependencies, include or omit the config file section, wire up the correct message types.

## Step 4 — Write the README.md

Every node needs a `README.md` with these sections:

```markdown
# <Package Name>

One paragraph describing what this node does and why it exists in the architecture.

## Topics

| Direction | Topic | Type | Rate |
|-----------|-------|------|------|
| Publishes | /foo/bar | std_msgs/String | 10 Hz |
| Subscribes | /params/VCON | interfaces/VCON | latched |

## Config

If the node loads a YAML file, describe each parameter, its type, and its default.

## Launch

```bash
# C++ implementation
ros2 launch <pkg> <node>_cpp.xml

# Python implementation
ros2 launch <pkg> <node>_py.xml
```

## Dependencies

- `interfaces` — custom message types
- `rclcpp` / `rclpy` — ROS2 client libraries
- any other dependencies
```

## Step 5 — Build and Verify

After creating all files, run:

```bash
colcon build --packages-select <pkg>
```

If the package uses `interfaces`, build both together:

```bash
colcon build --packages-select interfaces <pkg>
```

Fix any compiler or ament errors before reporting the node as done. If the build succeeds, source the workspace:

```bash
source install/setup.bash
```

## Key Rules (the Why)

**Config files go to share, never hardcoded.** The install tree for a colcon workspace separates source from install — any path that works during development will break in a deployed or Docker environment. Always install config files to `share/${PROJECT_NAME}` and look them up at runtime via the ament index.

**Python executables keep the `.py` suffix.** `install(PROGRAMS ...)` preserves the filename, so the launch file `exec` attribute must match exactly (e.g. `exec="my_node.py"`). Omitting `.py` will cause a "not found" error at launch.

**`ament_python_install_package` is not optional.** Without it, other nodes cannot `import` from this package's Python directory, even if they're in the same workspace.

**One launch file per executable.** Keeping them separate makes it trivial to launch just the C++ or just the Python version, and makes the launch files simpler to read.
