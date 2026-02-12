#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import VCON as VCON_msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class VehicleWireframeVisualizer(Node):
    def __init__(self):
        super().__init__('vehicle_visualizer')
        self.odom_sub = self.create_subscription(Odometry, "/odometry/ego", self.odom_callback, 10)
        self.vcon_sub = self.create_subscription(VCON_msg,"/params/VCON",self.vcon_callback,10)
        self.marker_pub = self.create_publisher(Marker, '/visualization/egoframe', 10)

        self.vcon = VCON_msg()
        self.vcon_received = False

    def vcon_callback(self, msg):
        self.vcon = msg
        self.vcon_received = True

    def odom_callback(self, msg):

        if not self.vcon_received:
            return

        marker = Marker() 
        marker.header = msg.header
        marker.ns = "vehicle_wireframe"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # The position/orientation of the center of the box
        marker.pose = msg.pose.pose
        if msg.header.frame_id == "rear_axle":
            marker.pose.position.x += self.vcon.vehicle_dimensions.wheelbase / 2
            
        elif msg.header.frame_id == "front_axle":
            marker.pose.position.x -= self.vcon.vehicle_dimensions.wheelbase / 2

        else:
            pass 
    
        # For LINE_LIST:
        # scale.x is the thickness of the lines
        marker.scale.x = 0.05 
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0 # Fully opaque lines

        # Calculate half-dimensions
        dx = self.vcon.vehicle_dimensions.length / 2.0
        dy = self.vcon.vehicle_dimensions.width / 2.0
        dz = self.vcon.vehicle_dimensions.height

        # Define the 8 vertices of the box relative to its center
        v = [
            Point(x=dx, y=dy, z=dz),   # 0: top front left
            Point(x=dx, y=-dy, z=dz),  # 1: top front right
            Point(x=-dx, y=-dy, z=dz), # 2: top back right
            Point(x=-dx, y=dy, z=dz),  # 3: top back left
            Point(x=dx, y=dy, z=0.0),  # 4: bottom front left
            Point(x=dx, y=-dy, z=0.0), # 5: bottom front right
            Point(x=-dx, y=-dy, z=0.0),# 6: bottom back right
            Point(x=-dx, y=dy, z=0.0)  # 7: bottom back left
        ]

        # Define the 12 edges (pairs of vertices)
        edges = [
            (0,1), (1,2), (2,3), (3,0), # Top face
            (4,5), (5,6), (6,7), (7,4), # Bottom face
            (0,4), (1,5), (2,6), (3,7)  # Vertical pillars
        ]

        for start, end in edges:
            marker.points.append(v[start])
            marker.points.append(v[end])

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleWireframeVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()