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
    '~/ros2_ws/src/simulators/webots_vehicle_sim/worlds/superb.wbt'
)

ROAD_SURFACE_Z = 0.10   # road sits 10 cm above the ground plane


def _sensor_rotation_vrml(pitch: float, yaw: float) -> str:
    """Return a VRML axis-angle rotation string for the given pitch + yaw.

    Pitch rotates around the Y axis (negative = nose down).
    Yaw rotates around the Z axis (positive = left).
    The combined quaternion is q_yaw * q_pitch so yaw is applied last.
    """
    cp2, sp2 = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy2, sy2 = math.cos(yaw   / 2.0), math.sin(yaw   / 2.0)

    # q_pitch (Y-axis): (x=0, y=sp2, z=0,   w=cp2)
    # q_yaw   (Z-axis): (x=0, y=0,   z=sy2, w=cy2)
    # combined = q_yaw * q_pitch
    px, py, pz, pw = 0.0, sp2, 0.0,  cp2
    qx, qy, qz, qw = 0.0, 0.0, sy2, cy2

    rx = qw*px + qx*pw + qy*pz - qz*py
    ry = qw*py - qx*pz + qy*pw + qz*px
    rz = qw*pz + qx*py - qy*px + qz*pw
    rw = qw*pw - qx*px - qy*py - qz*pz

    angle = 2.0 * math.acos(max(-1.0, min(1.0, rw)))
    s = math.sqrt(max(0.0, 1.0 - rw * rw))
    if s < 1e-6:
        return '0 0 1 0'
    return f'{rx/s:.6f} {ry/s:.6f} {rz/s:.6f} {angle:.6f}'


def generate_vehicle(msg) -> str:
    """Returns the VRML Robot block for the Skoda Superb, derived from VCON."""

    cam   = msg.front_camera
    radar = msg.front_radar

    length      = msg.vehicle_dimensions.length
    width       = msg.vehicle_dimensions.width
    height      = msg.vehicle_dimensions.height
    wheelbase   = msg.vehicle_dimensions.wheelbase
    track_width = msg.vehicle_dimensions.track_width

    wheel_radius    = msg.wheel_dimensions.wheel_radius
    wheel_thickness = msg.wheel_dimensions.tire_width
    wheel_mass      = msg.wheel_dimensions.wheel_mass

    max_motor_torque_per_wheel = 2100

    front_axle_x = wheelbase
    front_axle_z = wheel_radius

    rear_axle_x = 0
    rear_axle_z = wheel_radius

    cog_x = wheelbase / 2
    cog_y = 0
    cog_z = wheel_radius + height / 2

    left_y  =  track_width / 2.0
    right_y = -track_width / 2.0

    body_h  = height * 0.6
    cabin_h = height * 0.4
    cabin_w = width  * 0.90
    body_z  = wheel_radius + body_h  / 2
    cabin_z = wheel_radius + body_h  + cabin_h / 2

    ll_x  = length * 0.025
    ll_y  = width  * 0.18
    ll_z  = height * 0.09
    fl_x  = cog_x + length / 2 - ll_x / 2
    rl_x  = cog_x - length / 2 + ll_x / 2
    lgt_z = wheel_radius + body_h * 0.65
    lgt_y = width / 2 - ll_y / 2

    ws_slope  = 1.1
    rw_slope  = 0.9
    ws_dx     = cabin_h * ws_slope
    rw_dx     = cabin_h * rw_slope
    ws_slant  = cabin_h / math.cos(math.atan(ws_slope))
    rw_slant  = cabin_h / math.cos(math.atan(rw_slope))
    ws_angle  = -math.atan(ws_slope)
    rw_angle  =  math.atan(rw_slope)
    cabin_offset = -0.30

    ws_bot_x  = front_axle_x + cabin_offset
    rw_bot_x  = rear_axle_x  + cabin_offset
    ws_x      = ws_bot_x - ws_dx / 2
    rw_x      = rw_bot_x + rw_dx / 2
    ws_thick  = 0.025

    cabin_vis_l = wheelbase - ws_dx - rw_dx
    cabin_vis_x = (wheelbase + rw_dx - ws_dx) / 2 + cabin_offset

    body_top    = wheel_radius + body_h
    cabin_top   = body_top + cabin_h
    cabin_frt_x = ws_bot_x - ws_dx
    cabin_rr_x  = rw_bot_x + rw_dx
    hy          = cabin_w / 2

    mir_lx = width * 0.09
    mir_ly = width * 0.05
    mir_lz = height * 0.07
    mir_x  = ws_bot_x
    mir_y  = width / 2 + mir_ly / 2
    mir_z  = body_top + mir_lz / 2

    r, g, b = 0.13, 0.25, 0.13

    return (
        'DEF superb Robot {\n'
        f'  translation 0 0 {ROAD_SURFACE_Z + 0.02:.4f}\n'
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
        '\n'
        f'    Camera {{\n'
        f'      translation {cam.pose.x:.4f} {cam.pose.y:.4f} {cam.pose.z:.4f}\n'
        f'      rotation {_sensor_rotation_vrml(cam.pose.pitch, cam.pose.yaw)}\n'
        f'      name "front_camera"\n'
        f'      fieldOfView {cam.horizontal_fov:.4f}\n'
        f'      width {cam.width}\n'
        f'      height {cam.height}\n'
        f'      near {cam.near:.4f}\n'
        f'      far {cam.far:.1f}\n'
        f'    }}\n'
        '\n'
        f'    Radar {{\n'
        f'      translation {radar.pose.x:.4f} {radar.pose.y:.4f} {radar.pose.z:.4f}\n'
        f'      rotation {_sensor_rotation_vrml(radar.pose.pitch, radar.pose.yaw)}\n'
        f'      name "front_radar"\n'
        f'      minRange {radar.min_range:.1f}\n'
        f'      maxRange {radar.max_range:.1f}\n'
        f'      horizontalFieldOfView {radar.horizontal_fov:.4f}\n'
        f'      verticalFieldOfView {radar.vertical_fov:.4f}\n'
        f'    }}\n'
        '\n'
        '    DEF FRONT_LEFT_STEER HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 0 1\n'
        f'        anchor {front_axle_x:.4f} {left_y:.4f} {front_axle_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "front_left_steer"\n'
        f'          maxPosition {math.radians(30):.4f}\n'
        f'          minPosition {-math.radians(30):.4f}\n'
        '          maxVelocity 1.0\n'
        '          maxTorque 1000.0\n'
        '        }\n'
        '        PositionSensor {\n'
        '          name "front_left_steer_sensor"\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {front_axle_x:.4f} {left_y:.4f} {front_axle_z:.4f}\n'
        '        name "front_left_knuckle"\n'
        f'        boundingObject Sphere {{ radius 0.05 }}\n'
        f'        physics Physics {{ density -1 mass 1.0 }}\n'
        '        children [\n'
        '          DEF FRONT_LEFT_WHEEL HingeJoint {\n'
        '            jointParameters HingeJointParameters {\n'
        '              axis 0 1 0\n'
        '              anchor 0 0 0\n'
        '            }\n'
        '            device [\n'
        '              RotationalMotor {\n'
        '                name "front_left_motor"\n'
        '                maxVelocity 100\n'
        f'               maxTorque {max_motor_torque_per_wheel}\n'
        '              }\n'
        '              Brake {\n'
        '                name "front_left_brake"\n'
        '              }\n'
        '              PositionSensor {\n'
        '                name "front_left_sensor"\n'
        '              }\n'
        '            ]\n'
        '            endPoint Solid {\n'
        '              translation 0 0 0\n'
        '              rotation 1 0 0 -1.5708\n'
        '              children [\n'
        '                Shape {\n'
        '                  appearance PBRAppearance {\n'
        '                    baseColor 0.1 0.1 0.1\n'
        '                    roughness 0.9\n'
        '                  }\n'
        f'                  geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '                }\n'
        '                Shape {\n'
        '                  appearance PBRAppearance {\n'
        '                    baseColor 0.9 0.9 0.9\n'
        '                    roughness 0.5\n'
        '                  }\n'
        f'                  geometry Box {{ size {wheel_radius * 2:.4f} {wheel_thickness * 0.8:.4f} {wheel_radius * 0.15:.4f} }}\n'
        '                }\n'
        '              ]\n'
        '              name "front_left_wheel"\n'
        '              contactMaterial "wheel"\n'
        f'              boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        f'              physics Physics {{ density -1 mass {wheel_mass:.2f} }}\n'
        '            }\n'
        '          }\n'
        '        ]\n'
        '      }\n'
        '    }\n'
        '    DEF FRONT_RIGHT_STEER HingeJoint {\n'
        '      jointParameters HingeJointParameters {\n'
        '        axis 0 0 1\n'
        f'        anchor {front_axle_x:.4f} {right_y:.4f} {front_axle_z:.4f}\n'
        '      }\n'
        '      device [\n'
        '        RotationalMotor {\n'
        '          name "front_right_steer"\n'
        f'          maxPosition {math.radians(30):.4f}\n'
        f'          minPosition {-math.radians(30):.4f}\n'
        '          maxVelocity 1.0\n'
        '          maxTorque 1000.0\n'
        '        }\n'
        '        PositionSensor {\n'
        '          name "front_right_steer_sensor"\n'
        '        }\n'
        '      ]\n'
        '      endPoint Solid {\n'
        f'        translation {front_axle_x:.4f} {right_y:.4f} {front_axle_z:.4f}\n'
        '        name "front_right_knuckle"\n'
        f'        boundingObject Sphere {{ radius 0.05 }}\n'
        f'        physics Physics {{ density -1 mass 1.0 }}\n'
        '        children [\n'
        '          DEF FRONT_RIGHT_WHEEL HingeJoint {\n'
        '            jointParameters HingeJointParameters {\n'
        '              axis 0 1 0\n'
        '              anchor 0 0 0\n'
        '            }\n'
        '            device [\n'
        '              RotationalMotor {\n'
        '                name "front_right_motor"\n'
        '                maxVelocity 100\n'
        f'               maxTorque {max_motor_torque_per_wheel}\n'
        '              }\n'
        '              Brake {\n'
        '                name "front_right_brake"\n'
        '              }\n'
        '              PositionSensor {\n'
        '                name "front_right_sensor"\n'
        '              }\n'
        '            ]\n'
        '            endPoint Solid {\n'
        '              translation 0 0 0\n'
        '              rotation 1 0 0 -1.5708\n'
        '              children [\n'
        '                Shape {\n'
        '                  appearance PBRAppearance {\n'
        '                    baseColor 0.1 0.1 0.1\n'
        '                    roughness 0.9\n'
        '                  }\n'
        f'                  geometry Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        '                }\n'
        '                Shape {\n'
        '                  appearance PBRAppearance {\n'
        '                    baseColor 0.9 0.9 0.9\n'
        '                    roughness 0.5\n'
        '                  }\n'
        f'                  geometry Box {{ size {wheel_radius * 2:.4f} {wheel_thickness * 0.8:.4f} {wheel_radius * 0.15:.4f} }}\n'
        '                }\n'
        '              ]\n'
        '              name "front_right_wheel"\n'
        '              contactMaterial "wheel"\n'
        f'              boundingObject Cylinder {{ radius {wheel_radius:.4f} height {wheel_thickness:.4f} }}\n'
        f'              physics Physics {{ density -1 mass {wheel_mass:.2f} }}\n'
        '            }\n'
        '          }\n'
        '        ]\n'
        '      }\n'
        '    }\n'
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
        '        Brake {\n'
        '          name "rear_left_brake"\n'
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
        '        contactMaterial "wheel"\n'
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
        '        Brake {\n'
        '          name "rear_right_brake"\n'
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
        '        contactMaterial "wheel"\n'
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


def generate_world(msg) -> str:
    """Returns the complete Webots .wbt world string for the given VCON message."""

    return (
        '#VRML_SIM R2025a utf8\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/road/protos/StraightRoadSegment.proto"\n'
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/road/protos/RoadLine.proto"\n'
        '\n'
        'WorldInfo {\n'
        '  title "Vehicle Simulation"\n'
        '  basicTimeStep 32\n'
        '  coordinateSystem "ENU"\n'
        '  contactProperties [\n'
        '    ContactProperties {\n'
        '      coulombFriction [ 0.7 ]\n'
        '      rollingFriction 0.015 0.015 0\n'
        '    }\n'
        '    ContactProperties {\n'
        '      material1 "wheel"\n'
        '      coulombFriction [ 1.0 ]\n'
        '      softERP 0.8\n'
        '      softCFM 0.00001\n'
        '      rollingFriction 0.01 0.01 0\n'
        '    }\n'
        '  ]\n'
        '}\n'
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
        'Solid {\n'
        '  translation 0 0 -0.1\n'
        '  name "ground"\n'
        '  children [\n'
        '    Shape {\n'
        '      appearance PBRAppearance {\n'
        '        baseColor 0.25 0.45 0.15\n'
        '        roughness 1.0\n'
        '        metalness 0.0\n'
        '      }\n'
        '      geometry Box { size 2000 2000 0.2 }\n'
        '    }\n'
        '  ]\n'
        '  boundingObject Box { size 2000 2000 0.2 }\n'
        '}\n'
        '\n'
        f'Solid {{\n'
        f'  translation 0 0 {ROAD_SURFACE_Z - 0.5:.4f}\n'
        '  name "road_physics"\n'
        f'  boundingObject Box {{ size 2000 2000 1.0 }}\n'
        '}\n'
        '\n'
        f'StraightRoadSegment {{\n'
        f'  translation 0 0 {ROAD_SURFACE_Z}\n'
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
        '  length 1000\n'
        '  roadBoundingObject FALSE\n'
        '}\n'
        '\n'
    ) + generate_vehicle(msg)


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