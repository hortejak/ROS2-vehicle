import math
import rclpy
from interfaces.msg import WheelTicks, VCON


WHEEL_NAMES = [
    'front_left_sensor',
    'front_right_sensor',
    'rear_left_sensor',
    'rear_right_sensor',
]

TIMESTEP = 32


class WheelEncoderPlugin:

    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        self._ticks_per_rev = None  # will be set from VCON

        self._sensors = []
        for name in WHEEL_NAMES:
            sensor = self._robot.getDevice(name)
            sensor.enable(TIMESTEP)
            self._sensors.append(sensor)

        rclpy.init(args=None)
        self._node = rclpy.create_node('wheel_encoder_plugin')
        self._pub = self._node.create_publisher(WheelTicks, '/simulation/wheel/ticks', 10)
        self._sub = self._node.create_subscription(
            VCON,
            'params/VCON',
            self._vcon_callback,
            10
        )

        self._prev_radians = [0.0] * 4
        self._tick_counts  = [0]   * 4

        self._node.get_logger().info('WheelEncoderPlugin initialized — waiting for VCON ...')

    def _vcon_callback(self, msg: VCON):
        if self._ticks_per_rev is None:
            self._ticks_per_rev = msg.wheel_dimensions.ticks_per_revolution
            self._node.get_logger().info(
                f'WheelEncoderPlugin: ticks_per_rev={self._ticks_per_rev}'
            )

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)

        if self._ticks_per_rev is None:
            return  # wait until VCON received

        for i, sensor in enumerate(self._sensors):
            radians = sensor.getValue()
            delta_rad = radians - self._prev_radians[i]
            self._prev_radians[i] = radians
            delta_ticks = int(delta_rad / (2.0 * math.pi) * self._ticks_per_rev)
            self._tick_counts[i] += delta_ticks

        msg = WheelTicks()
        msg.front_left  = self._tick_counts[0]
        msg.front_right = self._tick_counts[1]
        msg.rear_left   = self._tick_counts[2]
        msg.rear_right  = self._tick_counts[3]
        self._pub.publish(msg)