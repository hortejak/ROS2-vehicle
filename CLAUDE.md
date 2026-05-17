# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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

- `VCON` — vehicle identity + `VehicleDimensions` + `WheelDimensions`
- `KinematicState` — x, y, theta, v
- `KinematicInput` — a (acceleration), delta (steering angle)
- `SimulationVehicleCmd` — per-wheel throttle/brake torques + steering_wheel angle
- `WheelTicks` — encoder ticks per wheel (FL/FR/RL/RR)

## Vehicle Configuration

Vehicle parameters are defined in `src/vehicle/vcon/VCON.yaml` (Skoda Superb Mk1):
- Wheelbase: 2.803 m, Track width: 1.515 m
- Wheel radius: 0.316 m, Ticks/rev: 512

Any code that needs vehicle geometry should subscribe to `/params/VCON` rather than hardcoding values.

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

## Current Status

`scenario_parking.xml` is the main launch file; parking-specific nodes (map, slot selection, goal, planner) are stubbed out and under development.
