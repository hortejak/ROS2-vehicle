import rclpy
from rclpy.node import Node
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

from nav_msgs.msg import OccupancyGrid

class MapCreatorFromFile(Node):

    def __init__(self):
        super().__init__("map_creator_from_file")
        
        self.resolution = 0.1  # resolution in meters (10 cm per cell)

        self.grid = None
        self.map_width = 0
        self.map_height = 0


        self.open_txt("oval_track.txt")

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

        print(self.grid.size)


def main(args=None):
    rclpy.init(args=args)
    node = MapCreatorFromFile()
    rclpy.spin(node)

    rclpy.shutdown()