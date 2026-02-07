#!/usr/bin/env python3

import rclpy
import json
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
        file_path = os.path.join(package_share_directory, 'VCON.json')
        self.load_vehicle_config(file_path)

        self.get_logger().info("VCON loaded")

        
        self.vcon_publisher = self.create_publisher(VCON_msg,"params/VCON",10)

        self.create_timer(dt,self.run)

    def load_vehicle_config(self, file_path):

        with open(file_path, 'r') as f:
            data = json.load(f)
            
        vehicle = data['vehicle']
        dims = vehicle['dimensions']

        self.vcon = VCON_msg()
        
        self.vcon.id = vehicle['metadata']['id']
        self.vcon.name = vehicle['metadata']['name']
        
        # Mapping the JSON to your nested message
        self.vcon.vehicle_dimensions.length = float(dims['length'])
        self.vcon.vehicle_dimensions.width = float(dims['width'])
        self.vcon.vehicle_dimensions.height = float(dims['height'])
        self.vcon.vehicle_dimensions.wheelbase = float(dims['wheelbase'])
        self.vcon.vehicle_dimensions.track_width = float(dims['track_width'])

    def run(self):       

        self.vcon_publisher.publish(self.vcon)

def main(args=None):
    rclpy.init()
    node = VCONPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()