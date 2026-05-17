"""
vehicle_controller_plugin.py

Webots ROS2 driver plugin that controls the vehicle.
Subscribes to /simulation/cmd (VehicleCmd) and:
  - Applies throttle to front wheels (FWD)
  - Applies Ackermann steering to front wheels
  - Applies brakes to all four wheels
"""

import math
import rclpy
from interfaces.msg import SimulationVehicleCmd, VCON


# TODO: later VCON value
MAX_STEERING_ANGLE  = math.radians(30.0)   # max wheel angle in radians
STEERING_RATIO      = 16.0                  # steering wheel deg / wheel deg
MAX_MOTOR_TORQUE    = 2100                 # Nm per front wheel
MAX_BRAKE_TORQUE    = 500.0                 # Nm per wheel

TIMESTEP = 32

ENGINE_RESISTANCE   =  200.0   # N — drivetrain drag when coasting (no gearbox, direct drive)
AERO_DRAG_COEFF      = 0.39    # 0.5 * rho * Cd * A


class VehicleControllerPlugin:

    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        try:
            rclpy.init(args=None)
        except:
            pass
        self._node = rclpy.create_node('vehicle_controller_plugin')
        # Steering motors
        self._steer_left  = self._robot.getDevice('front_left_steer')
        self._steer_right = self._robot.getDevice('front_right_steer')

        # Drive motors (FWD — front only)
        self._motor_fl = self._robot.getDevice('front_left_motor')
        self._motor_fr = self._robot.getDevice('front_right_motor')

        self._sensor_fl = self._robot.getDevice('front_left_sensor')
        self._sensor_fr = self._robot.getDevice('front_right_sensor')
        self._sensor_fl.enable(TIMESTEP)
        self._sensor_fr.enable(TIMESTEP)

        self._prev_pos_fl = 0.0
        self._prev_pos_fr = 0.0

        self._avg_vel = 0.0

        # Brakes
        self._brake_fl = self._robot.getDevice('front_left_brake')
        self._brake_fr = self._robot.getDevice('front_right_brake')
        self._brake_rl = self._robot.getDevice('rear_left_brake')
        self._brake_rr = self._robot.getDevice('rear_right_brake')

        # Steer sensors
        self._steer_sensor_left  = self._robot.getDevice('front_left_steer_sensor')
        self._steer_sensor_right = self._robot.getDevice('front_right_steer_sensor')
        self._steer_sensor_left.enable(TIMESTEP)
        self._steer_sensor_right.enable(TIMESTEP)

        self._motor_fl.enableForceFeedback(TIMESTEP)
        self._motor_fr.enableForceFeedback(TIMESTEP)

        self._node.get_logger().info('Controller plugin devices loaded.')

        # ROS2
        self._node.create_subscription(
            SimulationVehicleCmd,
            '/simulation/cmd',
            self._cmd_callback,
            10
        )
        self._node.create_subscription(
            VCON,
            'params/VCON',
            self._vcon_callback,
            10
        )

        self._wheelbase   = None
        self._track_width = None

        # latest command
        self._throttle    = 0.0
        self._steering_wheel = 0.0
        self._brake_fl_nm = 0.0
        self._brake_fr_nm = 0.0
        self._brake_rl_nm = 0.0
        self._brake_rr_nm = 0.0

        self._node.get_logger().info('VehicleControllerPlugin initialized')

    def _vcon_callback(self, msg: VCON):
        if self._wheelbase is None:
            self._wheelbase   = msg.vehicle_dimensions.wheelbase
            self._track_width = msg.vehicle_dimensions.track_width
            self._wheel_radius = msg.wheel_dimensions.wheel_radius
            self._node.get_logger().info(
                f'VehicleControllerPlugin: wheelbase={self._wheelbase:.3f} '
                f'track_width={self._track_width:.3f}'
            )

    def _cmd_callback(self, msg: SimulationVehicleCmd):
        self._throttle       = msg.throttle
        self._steering_wheel = msg.steering_wheel
        self._brake_fl_nm    = msg.brake_front_left
        self._brake_fr_nm    = msg.brake_front_right
        self._brake_rl_nm    = msg.brake_rear_left
        self._brake_rr_nm    = msg.brake_rear_right

    def _ackermann(self, steering_wheel_deg):
        """
        Convert steering wheel angle to individual wheel angles using Ackermann geometry.
        Returns (left_angle_rad, right_angle_rad)
        """
        if self._wheelbase is None or self._track_width is None:
            return 0.0, 0.0

        # steering wheel -> centre wheel angle via ratio
        centre_angle = math.radians(steering_wheel_deg / STEERING_RATIO)  # TODO: later VCON value

        # clamp to max steering angle
        centre_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, centre_angle))  # TODO: later VCON value

        if abs(centre_angle) < 1e-6:
            return 0.0, 0.0

        # Ackermann geometry
        # turning radius to centre of rear axle
        R = self._wheelbase / math.tan(abs(centre_angle))

        inner_angle = math.atan(self._wheelbase / (R - self._track_width / 2.0))
        outer_angle = math.atan(self._wheelbase / (R + self._track_width / 2.0))

        if centre_angle > 0:
            # turning left — left wheel is inner
            return inner_angle, outer_angle
        else:
            # turning right — right wheel is inner
            return -outer_angle, -inner_angle
        
    def _throttle_to_torque(self, throttle_pct):
        """
        Non-linear throttle to torque mapping for 2.0 TDI.
        Diesel characteristics:
        - Small response even at low throttle (no spark ignition delay)
        - Rapid torque build up to ~40%
        - Wide flat plateau from 40-80% (diesel torque band)
        - Slight drop off above 80% (fueling/boost limit)
        TODO: replace with full RPM-dependent map from VCON
        """
        if throttle_pct <= 0.0:
            return 0.0

        # normalize to 0-1
        t = throttle_pct / 100.0

        # piecewise curve matching diesel torque characteristics
        if t < 0.15:
            # 0-15%: slow initial response (turbo lag)
            factor = 0.3 * (t / 0.15) ** 2
        elif t < 0.40:
            # 15-40%: rapid torque build
            factor = 0.3 + 0.55 * ((t - 0.15) / 0.25)
        elif t < 0.80:
            # 40-80%: flat peak torque plateau (diesel sweet spot)
            factor = 0.85 + 0.10 * ((t - 0.40) / 0.40)
        else:
            # 80-100%: slight drop off
            factor = 0.95 + 0.05 * ((t - 0.80) / 0.20)

        return factor * MAX_MOTOR_TORQUE  # TODO: later VCON value

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)

        if self._wheelbase is None:
            return  # wait for VCON
        
        # --- Environemt ---

        pos_fl = self._sensor_fl.getValue()
        pos_fr = self._sensor_fr.getValue()
#        if pos_fl != self._prev_pos_fl or pos_fr != self._prev_pos_fr:
        vel_fl = abs(pos_fl - self._prev_pos_fl) / (TIMESTEP / 1000.0)
        vel_fr = abs(pos_fr - self._prev_pos_fr) / (TIMESTEP / 1000.0)
        avg_vel = (vel_fl + vel_fr) / 2.0
        self._prev_pos_fl = pos_fl
        self._prev_pos_fr = pos_fr
        vehicle_speed = avg_vel * self._wheel_radius  # rad/s * m = m/s

        resistance_aero_force = AERO_DRAG_COEFF * pow(vehicle_speed,2)
        resistance_engine_force = ENGINE_RESISTANCE

        # --- Steering ---
        left_angle, right_angle = self._ackermann(self._steering_wheel)
        self._steer_left.setPosition(left_angle)
        self._steer_right.setPosition(right_angle)

        # --- Throttle (FWD) ---
        # throttle 0-100 -> motor velocity 0-max_velocity
        # TODO: later VCON value for max velocity
        target_torque = self._throttle_to_torque(self._throttle)

        self._motor_fl.setTorque(target_torque)
        self._motor_fr.setTorque(target_torque)

        resistance_force = resistance_aero_force
        resistance_force += resistance_engine_force if self._throttle <= 0.01 else 0

        resistance_damping = resistance_force * pow(self._wheel_radius,2) / vehicle_speed if vehicle_speed >= 0.01 else 0

        # --- Brakes ---
        self._brake_fl.setDampingConstant(self._brake_fl_nm + resistance_damping/4)
        self._brake_fr.setDampingConstant(self._brake_fr_nm + resistance_damping/4)
        self._brake_rl.setDampingConstant(self._brake_rl_nm + resistance_damping/4)
        self._brake_rr.setDampingConstant(self._brake_rr_nm + resistance_damping/4)

#        self._node.get_logger().info(
#            f'target_engine_torque={target_torque:.1f} Nm/m '
#            f'wheel_ang_speed={avg_vel:.1f} rad/s '
#            f'velocity={vehicle_speed:.1f} m/s '
#            f'fl_brake={self._brake_fl_nm:.1f} Nm '
#            f'fr_brake={self._brake_fr_nm:.1f} Nm '
#            f'damping={resistance_damping:.1f} Nm*m/rad'
#        )