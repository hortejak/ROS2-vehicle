import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class baseLinkToWorldTransform(Node):

    def __init__(self):
        super().__init__('base_link_to_world_transform')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(PoseStamped,"/odometry/position",self.handle_odom,10)


    def handle_odom(self,msg):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = baseLinkToWorldTransform()
    rclpy.spin(node)

    rclpy.shutdown()