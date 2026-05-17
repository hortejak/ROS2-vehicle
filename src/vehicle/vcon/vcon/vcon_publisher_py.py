#!/usr/bin/env python3
import rclpy
import yaml
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from interfaces.msg import VCON as VCON_msg

class VCONPublisher(Node):
    def __init__(self):
        super().__init__("VCON_publisher_py")

        dt = 1      # publish 1 times per second

        self.get_logger().info("Loading VCON")

        package_share_directory = get_package_share_directory('vcon')
        file_path = os.path.join(package_share_directory, 'VCON.yaml')
        self.load_vehicle_config(file_path)

        self.get_logger().info("VCON loaded")

        
        self.vcon_publisher = self.create_publisher(VCON_msg,"params/VCON",10)

        self.create_timer(dt,self.run)

    def _load_sensor_pose(self, target_pose, cfg):
        target_pose.x     = float(cfg['x'])
        target_pose.y     = float(cfg['y'])
        target_pose.z     = float(cfg['z'])
        target_pose.pitch = float(cfg['pitch'])
        target_pose.yaw   = float(cfg['yaw'])

    def load_vehicle_config(self, file_path):

        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        vehicle = data['vehicle']
        dims = vehicle['dimensions']

        self.vcon = VCON_msg()

        self.vcon.id = vehicle['metadata']['id']
        self.vcon.name = vehicle['metadata']['name']

        self.vcon.vehicle_dimensions.length = float(dims['length'])
        self.vcon.vehicle_dimensions.width = float(dims['width'])
        self.vcon.vehicle_dimensions.height = float(dims['height'])
        self.vcon.vehicle_dimensions.wheelbase = float(dims['wheelbase'])
        self.vcon.vehicle_dimensions.track_width = float(dims['track_width'])

        self.vcon.wheel_dimensions.wheel_radius = float(dims['wheel_dimensions']['wheel_radius'])
        self.vcon.wheel_dimensions.tire_width = float(dims['wheel_dimensions']['tire_width'])
        self.vcon.wheel_dimensions.wheel_mass = float(dims['wheel_dimensions']['wheel_mass'])
        self.vcon.wheel_dimensions.ticks_per_revolution = float(dims['wheel_dimensions']['ticks_per_revolution'])

        cam = vehicle['sensors']['front_camera']
        self._load_sensor_pose(self.vcon.front_camera.pose, cam)
        self.vcon.front_camera.horizontal_fov = float(cam['horizontal_fov'])
        self.vcon.front_camera.width          = int(cam['width'])
        self.vcon.front_camera.height         = int(cam['height'])
        self.vcon.front_camera.near           = float(cam['near'])
        self.vcon.front_camera.far            = float(cam['far'])

        radar = vehicle['sensors']['front_radar']
        self._load_sensor_pose(self.vcon.front_radar.pose, radar)
        self.vcon.front_radar.horizontal_fov = float(radar['horizontal_fov'])
        self.vcon.front_radar.vertical_fov   = float(radar['vertical_fov'])
        self.vcon.front_radar.min_range      = float(radar['min_range'])
        self.vcon.front_radar.max_range      = float(radar['max_range'])

    def run(self):       

        self.vcon_publisher.publish(self.vcon)

def main(args=None):
    rclpy.init()
    node = VCONPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()