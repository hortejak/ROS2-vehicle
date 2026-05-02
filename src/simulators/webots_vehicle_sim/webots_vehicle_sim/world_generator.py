"""
world_generator.py

Subscribes to 'params/VCON', waits for the first message,
generates a Webots .wbt world file with correct vehicle dimensions,
then launches Webots with that world file.
"""

import math
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from interfaces.msg import VCON


WORLD_OUTPUT_PATH = os.path.expanduser(
    '~/ros2_ws/src/webots_vehicle_sim/worlds/superb.wbt'
)


def generate_world(msg) -> str:

    length      = msg.vehicle_dimensions.length
    width       = msg.vehicle_dimensions.width
    height      = msg.vehicle_dimensions.height
    wheelbase   = msg.vehicle_dimensions.wheelbase
    track_width = msg.vehicle_dimensions.track_width

    wheel_radius    = msg.wheel_dimensions.wheel_radius
    wheel_thickness = msg.wheel_dimensions.tire_width
    wheel_mass      = msg.wheel_dimensions.wheel_mass
    ticks_per_revolution = msg.wheel_dimensions.ticks_per_revolution

    front_axle_x = wheelbase
    front_axle_y = 0
    front_axle_z = wheel_radius

    rear_axle_x = 0
    rear_axle_y = 0
    rear_axle_z = wheel_radius

    cog_x = wheelbase/2
    cog_y = 0
    cog_z = wheel_radius + height/2

    left_y  =  track_width / 2.0
    right_y = -track_width / 2.0

    # sedan 3-box silhouette
    body_h  = height * 0.6                          # lower chassis height
    cabin_h = height * 0.4                         # cabin (passenger cell) height
    cabin_l = length * 0.44                          # cabin length
    cabin_w = width  * 0.90                          # cabin slightly narrower than body
    cabin_x = cog_x - length * 0.06                 # cabin biased slightly rearward
    body_z  = wheel_radius + body_h  / 2
    cabin_z = wheel_radius + body_h  + cabin_h / 2

    # lights (flush with front / rear face)
    ll_x  = length * 0.025                          # light depth (X)
    ll_y  = width  * 0.18                           # light width (Y)
    ll_z  = height * 0.09                           # light height (Z)
    fl_x  = cog_x + length / 2 - ll_x / 2          # front light center X
    rl_x  = cog_x - length / 2 + ll_x / 2          # rear  light center X
    lgt_z = wheel_radius + body_h * 0.65            # light center Z
    lgt_y = width / 2 - ll_y / 2                    # light center |Y| (outer edge)

    # windshields — slope expressed as dx/dz (horizontal set-back per unit rise)
    ws_slope  = 1.1                                     # front windshield (from vertical)
    rw_slope  = 0.9                                   # rear window  (from vertical)
    ws_dx     = cabin_h * ws_slope                       # horizontal set-back of top vs bottom
    rw_dx     = cabin_h * rw_slope
    ws_slant  = cabin_h / math.cos(math.atan(ws_slope))  # true slant height
    rw_slant  = cabin_h / math.cos(math.atan(rw_slope))
    ws_angle  = -math.atan(ws_slope)                     # rotation around Y (neg = top rearward)
    rw_angle  =  math.atan(rw_slope)                     # rotation around Y (pos = top forward)
    cabin_offset = -0.30                                 # shift whole cabin rearward

    # windshield bottoms shifted by cabin_offset
    ws_bot_x  = front_axle_x + cabin_offset             # windshield bottom X
    rw_bot_x  = rear_axle_x  + cabin_offset             # rear window bottom X
    ws_x      = ws_bot_x - ws_dx / 2                    # windshield center X
    rw_x      = rw_bot_x + rw_dx / 2                    # rear window center X
    ws_thick  = 0.025                                    # panel thickness

    # cabin box spans exactly between the two windshield tops
    cabin_vis_l = wheelbase - ws_dx - rw_dx
    cabin_vis_x = (wheelbase + rw_dx - ws_dx) / 2 + cabin_offset

    # pillar triangle fill geometry
    body_top    = wheel_radius + body_h
    cabin_top   = body_top + cabin_h
    cabin_frt_x = ws_bot_x - ws_dx    # cabin front face X = windshield top X
    cabin_rr_x  = rw_bot_x + rw_dx   # cabin rear  face X = rear-window top X
    hy          = cabin_w / 2         # half cabin width

    # side mirrors — mounted at A-pillar, beltline height
    mir_lx = width * 0.09      # ~16 cm in forward direction
    mir_ly = width * 0.05      # ~9 cm sticking out
    mir_lz = height * 0.07     # ~10 cm tall
    mir_x  = ws_bot_x          # at A-pillar / windshield base
    mir_y  = width / 2 + mir_ly / 2
    mir_z  = body_top + mir_lz / 2   # flush with body-cabin junction

    r,  g,  b  = 0.13, 0.25, 0.13

    world = (
        '#VRML_SIM R2025a utf8\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/road/protos/StraightRoadSegment.proto"\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/road/protos/RoadLine.proto"\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/floors/protos/Floor.proto"\n'
        '\n'
        'WorldInfo {\n'
        '  title "Vehicle Simulation"\n'
        '  basicTimeStep 16\n'
        '  coordinateSystem "ENU"\n'
        '}\n'
        '\n'
        'Viewpoint {\n'
        '  orientation 0 1 0 0.221\n'
        '  position -50 0 15\n'
        '  follow "Skoda_Superb_Mk1"\n'
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
        'Floor {\n'
        '  translation 0 0 0\n'
        '  size 500 500\n'
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
        '  roadBoundingObject TRUE\n'
        '}\n'
        '\n'
        'DEF superb Robot {\n'
        '  translation 0 0 0\n'
        '  rotation 0 0 1 0\n'
        '  name "Skoda_Superb_Mk1"\n'
        '  children [\n'
        '\n'
        f'    Transform {{\n'
        f'      translation {cog_x:.4f} 0 {body_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        f'            baseColor {r} {g} {b}\n'
        '            roughness 0.5\n'
        '            metalness 0.5\n'
        '          }\n'
        f'          geometry Box {{ size {length:.4f} {width:.4f} {body_h:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        '\n'
        f'    Transform {{\n'
        f'      translation {cabin_vis_x:.4f} 0 {cabin_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        f'            baseColor {r*0.85:.3f} {g*0.85:.3f} {b*0.85:.3f}\n'
        '            roughness 0.2\n'
        '            metalness 0.8\n'
        '          }\n'
        f'          geometry Box {{ size {cabin_vis_l:.4f} {cabin_w:.4f} {cabin_h:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        '\n'
        f'    Transform {{\n'
        f'      translation {ws_x:.4f} 0 {cabin_z:.4f}\n'
        f'      rotation 0 1 0 {ws_angle:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        '            baseColor 0.4 0.55 0.7\n'
        '            transparency 0.45\n'
        '            roughness 0.05\n'
        '            metalness 0.1\n'
        '          }\n'
        f'          geometry Box {{ size {ws_thick:.4f} {cabin_w:.4f} {ws_slant:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        '\n'
        f'    Transform {{\n'
        f'      translation {rw_x:.4f} 0 {cabin_z:.4f}\n'
        f'      rotation 0 1 0 {rw_angle:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        '            baseColor 0.4 0.55 0.7\n'
        '            transparency 0.45\n'
        '            roughness 0.05\n'
        '            metalness 0.1\n'
        '          }\n'
        f'          geometry Box {{ size {ws_thick:.4f} {cabin_w:.4f} {rw_slant:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        '\n'
        f'    Shape {{\n'
        f'      appearance PBRAppearance {{\n'
        f'        baseColor {r*0.85:.3f} {g*0.85:.3f} {b*0.85:.3f}\n'
        f'        roughness 0.2\n'
        f'        metalness 0.8\n'
        f'      }}\n'
        f'      geometry IndexedFaceSet {{\n'
        f'        coord Coordinate {{\n'
        f'          point [\n'
        f'            {ws_bot_x:.4f} { hy:.4f} {body_top:.4f}\n'
        f'            {cabin_frt_x:.4f}  { hy:.4f} {body_top:.4f}\n'
        f'            {cabin_frt_x:.4f}  { hy:.4f} {cabin_top:.4f}\n'
        f'            {ws_bot_x:.4f} {-hy:.4f} {body_top:.4f}\n'
        f'            {cabin_frt_x:.4f}  {-hy:.4f} {body_top:.4f}\n'
        f'            {cabin_frt_x:.4f}  {-hy:.4f} {cabin_top:.4f}\n'
        f'            {rw_bot_x:.4f}  { hy:.4f} {body_top:.4f}\n'
        f'            {cabin_rr_x:.4f}   { hy:.4f} {body_top:.4f}\n'
        f'            {cabin_rr_x:.4f}   { hy:.4f} {cabin_top:.4f}\n'
        f'            {rw_bot_x:.4f}  {-hy:.4f} {body_top:.4f}\n'
        f'            {cabin_rr_x:.4f}   {-hy:.4f} {body_top:.4f}\n'
        f'            {cabin_rr_x:.4f}   {-hy:.4f} {cabin_top:.4f}\n'
        f'          ]\n'
        f'        }}\n'
        f'        coordIndex [\n'
        f'          0 1 2 -1\n'
        f'          3 5 4 -1\n'
        f'          6 8 7 -1\n'
        f'          9 10 11 -1\n'
        f'        ]\n'
        f'      }}\n'
        f'    }}\n'
        '\n'
        f'    Transform {{\n'
        f'      translation {fl_x:.4f} {lgt_y:.4f} {lgt_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        '            baseColor 1.0 1.0 0.8\n'
        '            roughness 0.05\n'
        '            metalness 0.0\n'
        '            emissiveColor 0.6 0.6 0.3\n'
        '          }\n'
        f'          geometry Box {{ size {ll_x:.4f} {ll_y:.4f} {ll_z:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        f'    Transform {{\n'
        f'      translation {fl_x:.4f} {-lgt_y:.4f} {lgt_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        '            baseColor 1.0 1.0 0.8\n'
        '            roughness 0.05\n'
        '            metalness 0.0\n'
        '            emissiveColor 0.6 0.6 0.3\n'
        '          }\n'
        f'          geometry Box {{ size {ll_x:.4f} {ll_y:.4f} {ll_z:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        '\n'
        f'    Transform {{\n'
        f'      translation {rl_x:.4f} {lgt_y:.4f} {lgt_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        '            baseColor 0.8 0.05 0.05\n'
        '            roughness 0.05\n'
        '            metalness 0.0\n'
        '            emissiveColor 0.4 0.0 0.0\n'
        '          }\n'
        f'          geometry Box {{ size {ll_x:.4f} {ll_y:.4f} {ll_z:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        f'    Transform {{\n'
        f'      translation {rl_x:.4f} {-lgt_y:.4f} {lgt_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        '            baseColor 0.8 0.05 0.05\n'
        '            roughness 0.05\n'
        '            metalness 0.0\n'
        '            emissiveColor 0.4 0.0 0.0\n'
        '          }\n'
        f'          geometry Box {{ size {ll_x:.4f} {ll_y:.4f} {ll_z:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        '\n'
        f'    Transform {{\n'
        f'      translation {mir_x:.4f} {mir_y:.4f} {mir_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        f'            baseColor {r} {g} {b}\n'
        '            roughness 0.4\n'
        '          }\n'
        f'          geometry Box {{ size {mir_lx:.4f} {mir_ly:.4f} {mir_lz:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        f'    Transform {{\n'
        f'      translation {mir_x:.4f} {-mir_y:.4f} {mir_z:.4f}\n'
        '      children [\n'
        '        Shape {\n'
        '          appearance PBRAppearance {\n'
        f'            baseColor {r} {g} {b}\n'
        '            roughness 0.4\n'
        '          }\n'
        f'          geometry Box {{ size {mir_lx:.4f} {mir_ly:.4f} {mir_lz:.4f} }}\n'
        '        }\n'
        '      ]\n'
        '    }\n'
        '\n'
        '    GPS {\n'
        '      name "gps"\n'
        '    }\n'
        '    InertialUnit {\n'
        '      name "inertial_unit"\n'
        '    }\n'
        '    Gyro {\n'
        '      name "gyro"\n'
        '    }\n'
        '    Accelerometer {\n'
        '      name "accelerometer"\n'
        '    }\n'
        '    DEF FRONT_LEFT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 1 0\n'
        f'        anchor {front_axle_x:.4f} {left_y:.4f} {front_axle_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "front_left_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '        PositionSensor {\n'
        '          name "front_left_sensor"\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {front_axle_x:.4f} {left_y:.4f} {front_axle_z:.4f}\n'
        '        rotation 1 0 0 -1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.9 0.9 0.9\n'
        '              roughness 0.5\n'
        '            }\n'
        f'            geometry Box {{ size {wheel_radius * 2:.4f} {wheel_thickness * 0.8:.4f} {wheel_radius * 0.15:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "front_left_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        f'        physics Physics {{ density -1 mass {wheel_mass:.2f} }}\n'
        '      }\n'
        '    }\n'
        '\n'
        '    DEF FRONT_RIGHT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 1 0\n'
        f'        anchor {front_axle_x:.4f} {right_y:.4f} {front_axle_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "front_right_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '        PositionSensor {\n'
        '          name "front_right_sensor"\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {front_axle_x:.4f} {right_y:.4f} {front_axle_z:.4f}\n'
        '        rotation 1 0 0 -1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.9 0.9 0.9\n'
        '              roughness 0.5\n'
        '            }\n'
        f'            geometry Box {{ size {wheel_radius * 2:.4f} {wheel_thickness * 0.8:.4f} {wheel_radius * 0.15:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "front_right_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        f'        physics Physics {{ density -1 mass {wheel_mass:.2f} }}\n'
        '      }\n'
        '    }\n'
        '\n'
        '    DEF REAR_LEFT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 1 0\n'
        f'        anchor {rear_axle_x:.4f} {left_y:.4f} {rear_axle_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "rear_left_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '        PositionSensor {\n'
        '          name "rear_left_sensor"\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {rear_axle_x:.4f} {left_y:.4f} {rear_axle_z:.4f}\n'
        '        rotation 1 0 0 -1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.9 0.9 0.9\n'
        '              roughness 0.5\n'
        '            }\n'
        f'            geometry Box {{ size {wheel_radius * 2:.4f} {wheel_thickness * 0.8:.4f} {wheel_radius * 0.15:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "rear_left_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        f'        physics Physics {{ density -1 mass {wheel_mass:.2f} }}\n'
        '      }\n'
        '    }\n'
        '\n'
        '    DEF REAR_RIGHT_WHEEL HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 1 0\n'
        f'        anchor {rear_axle_x:.4f} {right_y:.4f} {rear_axle_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "rear_right_motor"\n'
        '          maxVelocity 100\n'
        '        }\n'
        '        PositionSensor {\n'
        '          name "rear_right_sensor"\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {rear_axle_x:.4f} {right_y:.4f} {rear_axle_z:.4f}\n'
        '        rotation 1 0 0 -1.5708\n'
        '        children [\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.1 0.1 0.1\n'
        '              roughness 0.9\n'
        '            }\n'
        f'            geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '          }\n'
        '          Shape {\n'
        '            appearance PBRAppearance {\n'
        '              baseColor 0.9 0.9 0.9\n'
        '              roughness 0.5\n'
        '            }\n'
        f'            geometry Box {{ size {wheel_radius * 2:.4f} {wheel_thickness * 0.8:.4f} {wheel_radius * 0.15:.4f} }}\n'
        '          }\n'
        '        ]\n'
        '        name "rear_right_wheel"\n'
        f'        boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        f'        physics Physics {{ density -1 mass {wheel_mass:.2f} }}\n'
        '      }\n'
        '    }\n'
        '\n'
        '  ]\n'
        '\n'
        f'  boundingObject Transform {{\n'
        f'    translation {cog_x:.4f} {cog_y:.4f} {cog_z:.4f}\n'
        f'    children [ Box {{ size {length:.4f} {width:.4f} {height:.4f} }} ]\n'
        '  }\n'
        '\n'
        '  physics Physics {\n'
        '    density -1\n'
        '    mass 1450\n'
        f'    centerOfMass [ {cog_x:.4f} {cog_y:.4f} {(cog_z - height * 0.1):.4f} ]\n'
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
        veh_dims = msg.vehicle_dimensions
        wheel_dims = msg.wheel_dimensions

        self.get_logger().info(
            f'Received VCON: "{msg.name}" id={msg.id} | '
            f'L={veh_dims.length:.3f} W={veh_dims.width:.3f} H={veh_dims.height:.3f} | '
            f'WB={veh_dims.wheelbase:.3f} TW={veh_dims.track_width:.3f} | '
            f'WR={wheel_dims.wheel_radius:.3f} TW={wheel_dims.tire_width:.3f} | '
            f'WM={wheel_dims.wheel_mass:.3f} TR={wheel_dims.ticks_per_revolution}'
        )

        world_content = generate_world(msg)

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