import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from interfaces.msg import VCON as VCON_msg
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class axlesToWorldTransform(Node):

    def __init__(self):
        super().__init__('axles_to_world_transform')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(PoseStamped,"/odometry/position",self.handle_odom,10)
        self.vcon_sub = self.create_subscription(VCON_msg,"/params/VCON",self.handle_vcon,10)

        self.d_translation_x = 0

    def handle_vcon(self,msg):

        self.d_translation_x = msg.vehicle_dimensions.wheelbase/2

    def handle_odom(self,msg):
 
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'rear_axle'

        if msg.header.frame_id == "rear_axle":
            dx = [0,self.d_translation_x,2*self.d_translation_x]
        elif msg.header.frame_id == "front_axle":
            dx = [-2*self.d_translation_x,-self.d_translation_x,0]
        else:
            dx = [-self.d_translation_x,0,self.d_translation_x]

        t.transform.translation.x = msg.pose.position.x + dx[0]
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        t.child_frame_id = 'center_of_gravity'
        t.transform.translation.x = msg.pose.position.x + dx[1]        
        self.tf_broadcaster.sendTransform(t)

        t.child_frame_id = 'front_axle'
        t.transform.translation.x = msg.pose.position.x + dx[2]
        self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = axlesToWorldTransform()
    rclpy.spin(node)

    rclpy.shutdown()