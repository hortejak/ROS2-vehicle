# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## General Instructions

- Always use Context7 when working with external libraries or frameworks.
- Once the node change has finished, try building it using colcon build --packages-select <package_name> and check for errors, if there are any, handle them.

## Build & Run

```bash
# Build all packages
./build.sh  # runs: colcon build --symlink-install

# Source the workspace after building
source install/setup.bash

# Run the main parking scenario (kinematic sim + RViz)
ros2 launch launch_files scenario_parking.xml

# Run the Webots physics simulation
ros2 launch webots_vehicle_sim vehicle_sim_launch.py
```

## Testing

```bash
# Run all tests
colcon test

# Run tests for a specific package
colcon test --packages-select <package_name>

# Run a single Python test file
python3 -m pytest src/<pkg>/test/test_*.py -v
```

## Package Architecture

The workspace is a ROS2-based autonomous vehicle platform simulating a Skoda Superb sedan. Packages are organized by function:

**Core data flow:**
```
VCON.yaml → [vcon_publisher] → /params/VCON
                                    ├─ [kinematic_model_py]   → /state/kinematic, /odometry/ego
                                    ├─ [rviz_vehicle_visualizator] → /visualization/egoframe
                                    └─ [webots_vehicle_sim]   (physics engine)

[map_creator] → /map/file → [center_line] → /planning/path

/simulation/cmd → Webots vehicle_controller_plugin → wheel torques/steering
```

**Packages:**

| Package | Role |
|---------|------|
| `interfaces` | All custom message types (VCON, KinematicState, KinematicInput, SimulationVehicleCmd, WheelTicks, etc.) |
| `vehicle/vcon` | Loads `VCON.yaml` and publishes vehicle config at 1 Hz to `/params/VCON` (C++ and Python implementations) |
| `vehicle_models/kinematic_model_py` | Kinematic bicycle model; subscribes to `/control/kinematic_input`, publishes state and odometry |
| `vehicle_controllers/general_controllers_py` | PID controller class (used as a library by other controllers) |
| `vehicle_controllers/longitudinal_control_py` | Longitudinal speed control via PID |
| `planning/center_line` | Extracts drivable centerline from OccupancyGrid using Numba-JIT connected-component inflation |
| `simulators/webots_vehicle_sim` | Webots physics sim: world generation, Ackermann steering, longitudinal dynamics (FWD), wheel encoders, GPS, IMU |
| `rviz_vehicle_visualizator` | Publishes LINE_LIST marker to visualize the vehicle box + wheels in RViz |
| `misc/tfs` | TF2 static/dynamic frame broadcasters (rear_axle, COG, front_axle, base_link, grid) |
| `misc/map_creator` | Publishes OccupancyGrid from text map files |
| `launch_files` | Top-level launch orchestration |

## Key Interfaces

All custom messages live in `src/interfaces/msg/`:

- `VCON` — full vehicle config (see VCON Message Structure below)
- `KinematicState` — x, y, theta, v
- `KinematicInput` — a (acceleration), delta (steering angle)
- `SimulationVehicleCmd` — per-wheel throttle/brake torques + steering_wheel angle
- `WheelTicks` — encoder ticks per wheel (FL/FR/RL/RR)
- `SensorPose` — x, y, z, pitch, yaw in vehicle frame (rear-axle origin)
- `CameraConfig` — SensorPose pose + horizontal_fov, width, height, near, far
- `RadarConfig` — SensorPose pose + horizontal_fov, vertical_fov, min_range, max_range

### VCON Message Structure

```
VCON
  string name / uint16 id
  VehicleDimensions   vehicle_dimensions  (length, width, height, wheelbase, track_width)
  WheelDimensions     wheel_dimensions    (wheel_radius, tire_width, wheel_mass, ticks_per_revolution)
  CameraConfig        front_camera
  RadarConfig         front_radar
```

`SensorPose` origin = rear axle centre at ground. +x forward, +y left, +z up (ENU). Pitch nose-down negative, yaw left positive, both in radians.

### Adding a new message to `interfaces`

Four files must change together every time:
1. Create `src/interfaces/msg/NewMsg.msg`
2. Add it to `rosidl_generate_interfaces(...)` in `src/interfaces/CMakeLists.txt` (intra-package deps resolve automatically; only external packages need `DEPENDENCIES`)
3. Load and assign in `src/vehicle/vcon/vcon/vcon_publisher_py.py`
4. Load and assign in `src/vehicle/vcon/src/vcon_publisher_cpp.cpp`

Then build: `colcon build --packages-select interfaces vcon`

## Vehicle Configuration

Vehicle parameters are defined in `src/vehicle/vcon/VCON.yaml` (Skoda Superb Mk1):
- Wheelbase: 2.803 m, Track width: 1.515 m
- Wheel radius: 0.316 m, Ticks/rev: 512
- Front camera: x=2.50, z=1.55, pitch=−0.15 rad, FOV=1.20 rad (70°), 640×480
- Front radar: x=3.70, z=0.50, FOV=0.26 rad (15°), range 3–150 m

Any code that needs vehicle geometry or sensor configuration should subscribe to `/params/VCON` rather than hardcoding values.

## Kinematic Model Origins

The kinematic model supports three reference point origins (set via launch arg `origin`):
- `RA` — rear axle (default in `scenario_parking.xml`)
- `FA` — front axle
- `COG` — center of gravity

## Webots Simulation Notes

- `world_generator.py` regenerates `worlds/superb.wbt` from VCON parameters at runtime (1.5 s delay)
- Webots driver starts 6 s after launch to allow world generation to complete
- Vehicle uses FWD motor configuration; steering uses `WheelLinkPositionSensor`
- Motor constants: MAX_MOTOR_TORQUE = 2100 Nm/wheel, MAX_BRAKE = 500 Nm
- Sensor positions/params are read from VCON (`msg.front_camera`, `msg.front_radar`) — never hardcoded
- `_sensor_rotation_vrml(pitch, yaw)` converts pitch+yaw angles to a VRML axis-angle string (q_yaw * q_pitch); lives in `world_generator.py`
- Onboard sensors: GPS, InertialUnit, Gyro, Accelerometer, 4× wheel PositionSensor, 2× steer PositionSensor, Camera (`front_camera`), Radar (`front_radar`)

## Current Status

`scenario_parking.xml` is the main launch file; parking-specific nodes (map, slot selection, goal, planner) are stubbed out and under development.
