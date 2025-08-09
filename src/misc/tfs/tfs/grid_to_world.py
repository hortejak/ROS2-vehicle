import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class gridToWorld(Node):

    def __init__(self):
        super().__init__('grid_to_world_transform')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(OccupancyGrid,"/map/file",self.map_cb,10)

    def map_cb(self,msg):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'grid'

        t.transform.translation.x = msg.info.origin.position.x
        t.transform.translation.y = msg.info.origin.position.y
        t.transform.translation.z = msg.info.origin.position.z

        t.transform.rotation = msg.info.origin.orientation
        #t.transform.rotation.w = - t.transform.rotation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = gridToWorld()
    rclpy.spin(node)

    rclpy.shutdown()