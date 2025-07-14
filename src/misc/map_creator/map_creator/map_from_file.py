import rclpy
from rclpy.node import Node
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class MapCreatorFromFile(Node):

    def __init__(self):
        super().__init__("map_creator_from_file")
        
        self.resolution = 0.1  # resolution in meters (10 cm per cell)

        self.grid = None
        self.map_width = 0
        self.map_height = 0

        frequency = 1


        self.open_txt("oval_track.txt")
        self.map_publisher = self.create_publisher(OccupancyGrid,"/map/file",10)
        self.create_timer(1/frequency,self.run)

    def open_txt(self,file_name):

        map_path = os.path.join(
            get_package_share_directory('map_creator'),
            'maps',
            file_name
        )

        with open(map_path,'r') as f:
            lines = f.readlines()

        grid = []
        for line in lines:
            row = [int(val) for val in line.strip().split()]
            grid.append(row)

        self.grid = np.array(grid)

    def run(self):

        o = OccupancyGrid()

        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "world"

        o.info.resolution = self.resolution
        o.info.height = self.grid.shape[0]
        o.info.width = self.grid.shape[1]
        origin = Pose()
        origin.position.x = self.resolution * self.grid.shape[0] / 2
        origin.position.y = -self.resolution * self.grid.shape[1] / 2

        origin.orientation.w = 0.707
        origin.orientation.z = 0.707

        o.info.origin = origin
        o.data = self.grid.astype(np.int8).flatten()

        self.map_publisher.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = MapCreatorFromFile()
    rclpy.spin(node)

    rclpy.shutdown()