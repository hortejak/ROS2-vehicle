import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

class MapCreatorFromFile(Node):

    def __init__(self):
        super().__init__("map_creator_from_file")


def main(args=None):
    rclpy.init(args=args)
    node = MapCreatorFromFile()
    rclpy.spin(node)

    rclpy.shutdown()