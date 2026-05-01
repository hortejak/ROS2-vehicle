"""
world_generator.py

Subscribes to 'params/VCON', waits for the first message,
generates a Webots .wbt world file with correct vehicle dimensions,
then launches Webots with that world file.
"""

import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from interfaces.msg import VCON


WORLD_OUTPUT_PATH = os.path.expanduser(
    '~/ros2_ws/src/webots_vehicle_sim/worlds/superb.wbt'
)


def generate_world(dims) -> str:

    length      = dims.length
    width       = dims.width
    height      = dims.height
    wheelbase   = dims.wheelbase
    track_width = dims.track_width

    wheel_radius    = height * 0.155
    wheel_thickness = width  * 0.085

    front_x = wheelbase / 2.0
    rear_x  = -wheelbase / 2.0

    left_z  = track_width / 2.0
    right_z = -track_width / 2.0

    # Vertical positions (Y axis = up in NUE)
    wheel_y  = wheel_radius                    # wheel center sits on ground
    body_y   = wheel_radius + height / 2.0    # body center above ground

    vehicle_y = wheel_y

    roof_length = length * 0.60
    roof_width  = width  * 0.92
    roof_height = height * 0.30

    r,  g,  b  = 0.13, 0.25, 0.13
    gr, gg, gb = 0.15, 0.20, 0.25

    vehicle_y = wheel_radius + 0.01

    world = (
        '#VRML_SIM R2025a utf8\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/road/protos/StraightRoadSegment.proto"\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/road/protos/RoadLine.proto"\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/floors/protos/RectangleArena.proto"\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/appearances/protos/Asphalt.proto"\n'
        '\n'
        'WorldInfo {\n'
        '  title "Vehicle Simulation"\n'
        '  basicTimeStep 16\n'
        '  coordinateSystem "NUE"\n'
        '}\n'
        '\n'
        'Viewpoint {\n'
        '  orientation 0 1 0 1.5708\n'
        '  position 0 8 20\n'
        '  follow "Skoda Superb Mk1"\n'
        '  followType "Tracking Shot"\n'
        '}\n'
        '\n'
        'Background {\n'
        '  skyColor [\n'
        '    0.4 0.7 1.0\n'
        '    0.4 0.7 1.0\n'
        '    0.4 0.7 1.0\n'
        '    0.4 0.7 1.0\n'
        '    0.4 0.7 1.0\n'
        '    0.4 0.7 1.0\n'
        '  ]\n'
        '}\n'
        '\n'
        'DirectionalLight {\n'
        '  ambientIntensity 1\n'
        '  direction 0.1 -1 -0.5\n'
        '  intensity 1\n'
        '}\n'
        '\n'
        'RectangleArena {\n'
        '  floorSize 500 500\n'
        '  floorTileSize 500 500\n'
        '  floorAppearance Asphalt {}\n'
        '  wallHeight 0.1\n'
        '}\n'
        '\n'
        'StraightRoadSegment {\n'
        '  translation 0 0 0\n'
        '  name "main_road"\n'
        '  width 8\n'
        '  numberOfLanes 2\n'
        '  numberOfForwardLanes 1\n'
        '  lines [\n'
        '    RoadLine {\n'
        '      color 0.85 0.85 0.0\n'
        '      type "continuous"\n'
        '      width 0.15\n'
        '    }\n'
        '  ]\n'
        '  length 200\n'
        '  roadBoundingObject FALSE\n'
        '}\n'
        '\n'
        'DEF superb Robot {\n'
        f'  translation 0 {vehicle_y:.4f} 0\n'
        '  rotation 0 1 0 0\n'
        '  name "Skoda Superb Mk1"\n'
        '  children [\n'
        '\n'
        '    Shape {\n'
        '      appearance PBRAppearance {\n'
        f'        baseColor {r} {g} {b}\n'
        '        roughness 0.4\n'
        '        metalness 0.6\n'
        '      }\n'
        f'      geometry Box {{ size {length:.4f} {height:.4f} {width:.4f} }}\n'
        '    }\n'
        '\n'
        '    DEF FRONT_LEFT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 0 1\n'
        f'        anchor {front_x:.4f} {wheel_y:.4f} {left_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "front_left_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {front_x:.4f} {wheel_y:.4f} {left_z:.4f}\n'
        '        rotation 1 0 0 1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "front_left_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '        physics Physics { density -1 mass 12 }\n'
        '      }\n'
        '    }\n'
        '\n'
        '    DEF FRONT_RIGHT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 0 1\n'
        f'        anchor {front_x:.4f} {wheel_y:.4f} {right_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "front_right_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {front_x:.4f} {wheel_y:.4f} {right_z:.4f}\n'
        '        rotation 1 0 0 1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "front_right_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '        physics Physics { density -1 mass 12 }\n'
        '      }\n'
        '    }\n'
        '\n'
        '    DEF REAR_LEFT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 0 1\n'
        f'        anchor {rear_x:.4f} {wheel_y:.4f} {left_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "rear_left_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {rear_x:.4f} {wheel_y:.4f} {left_z:.4f}\n'
        '        rotation 1 0 0 1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "rear_left_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '        physics Physics { density -1 mass 12 }\n'
        '      }\n'
        '    }\n'
        '\n'
        '    DEF REAR_RIGHT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 0 1\n'
        f'        anchor {rear_x:.4f} {wheel_y:.4f} {right_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "rear_right_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {rear_x:.4f} {wheel_y:.4f} {right_z:.4f}\n'
        '        rotation 1 0 0 1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "rear_right_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '        physics Physics { density -1 mass 12 }\n'
        '      }\n'
        '    }\n'
        '\n'
        '  ]\n'
        '\n'
        f'  boundingObject Box {{ size {length:.4f} {height:.4f} {width:.4f} }}\n'
        '\n'
        '  physics Physics {\n'
        '    density -1\n'
        '    mass 1450\n'
        f'    centerOfMass [ 0 {(-height * 0.1):.4f} 0 ]\n'
        '  }\n'
        '\n'
        '  controller "<extern>"\n'
        '}\n'
    )

    return world


class WorldGeneratorNode(Node):

    def __init__(self):
        super().__init__('world_generator')
        self._generated = False
        self._sub = self.create_subscription(
            VCON,
            'params/VCON',
            self._vcon_callback,
            10
        )
        self.get_logger().info(
            'WorldGenerator: waiting for VCON on params/VCON ...'
        )

    def _vcon_callback(self, msg: VCON):
        if self._generated:
            return

        self._generated = True
        dims = msg.vehicle_dimensions

        self.get_logger().info(
            f'Received VCON: "{msg.name}" id={msg.id} | '
            f'L={dims.length:.3f} W={dims.width:.3f} H={dims.height:.3f} | '
            f'WB={dims.wheelbase:.3f} TW={dims.track_width:.3f}'
        )

        world_content = generate_world(dims)

        os.makedirs(os.path.dirname(WORLD_OUTPUT_PATH), exist_ok=True)
        with open(WORLD_OUTPUT_PATH, 'w') as f:
            f.write(world_content)

        self.get_logger().info(f'World written to {WORLD_OUTPUT_PATH}')
        self.get_logger().info('Launching Webots ...')

        subprocess.Popen(
            [os.path.expanduser('~/.ros/webotsR2025a/webots/webots'), '--stdout', '--stderr', WORLD_OUTPUT_PATH],
            stdout=sys.stdout,
            stderr=sys.stderr
        )


def main(args=None):
    rclpy.init(args=args)
    node = WorldGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()