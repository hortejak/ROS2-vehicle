#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import KinematicState, KinematicInput
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import numpy as np

class KinematicModel(Node):

    # origin point is center of gravity

    def __init__(self):
        super().__init__("kinematic_model_py")

        # params
        self.a_min = -5
        self.a_max = 5
        a_rate_limit_per_s = 5
        self.lf = 1.4
        self.lr = 1.4

        dt = 0.05
        self.a_rate_limit = a_rate_limit_per_s * dt

        self.declare_parameter('origin_x',0.0)
        self.declare_parameter('origin_y',0.0)
        self.declare_parameter('origin_theta',0.0)

        origin_x = self.get_parameter('origin_x').get_parameter_value().double_value
        origin_y = self.get_parameter('origin_y').get_parameter_value().double_value
        origin_theta = self.get_parameter('origin_theta').get_parameter_value().double_value

        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.X = np.array([origin_x,origin_y,origin_theta,0])
        self.u = np.zeros(2)

        self.pose_publisher_seq = 0

        self.get_logger().info("Kinematic model starting")
        self.get_logger().info(f'Beginning from x={origin_x}; y={origin_y}; theta={origin_theta}')
        self.create_timer(dt,self.run)

        self.state_publisher = self.create_publisher(KinematicState,'state/kinematic',10)
        self.pose_publisher = self.create_publisher(PoseStamped,"/odometry/position",10)

        self.input_subscriber = self.create_subscription(KinematicInput,"input/kinematic",self.get_input,10)

        
    def get_input(self, msg):

        da = np.clip(msg.a - self.u[0],a_min=-self.a_rate_limit,a_max=self.a_rate_limit)
        self.u[0] = np.clip(self.u[0]+da, a_min=self.a_min,a_max=self.a_max)
        self.u[1] = msg.delta

    def compute_derivatives(self):
        """
        X ..... [x; y; theta; v]

        u ..... [a; delta]

        dX ..... [dx; dy; dtheta; dv] 
        """

        dX = np.zeros(4)

        beta = np.arctan(self.lr*np.tan(self.u[1])/(self.lf+self.lr))
        
        dX[0] = self.X[3] * np.cos(self.X[2]+beta)
        dX[1] = self.X[3] * np.sin(self.X[2]+beta)
        dX[2] = self.X[3] * np.cos(beta) * np.tan(self.u[1]) / (self.lf + self.lr)
        dX[3] = self.u[0]

        return dX
    
    def euler2quaternion2D(self,theta):

        q = np.zeros(4)

        q[0] = np.cos(theta/2)
        q[3] = np.sin(theta/2)

        return q
    
    def run(self):

        # TODO: improve by not taking u for the whole time but have 2 counters and 2 computations

        # get time
        t = self.get_clock().now().nanoseconds / 1e9
        dt = t - self.t0
        self.t0 = t
        #calculate the addition during this time
        self.X = self.X + self.compute_derivatives() * dt

        # create messages
        ks = KinematicState()
        ks.x = self.X[0]
        ks.y = self.X[1]
        ks.theta = self.X[2]
        ks.v = self.X[3]

        self.state_publisher.publish(ks)

        quat = self.euler2quaternion2D(self.X[2])

        q = Quaternion()
        q.w = quat[0]
        q.x = quat[1]
        q.y = quat[2]
        q.z = quat[3]

        p = Point()
        p.x = self.X[0]
        p.y = self.X[1]
        p.z = 0.0

        pose = Pose()
        pose.position = p
        pose.orientation = q

        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = "map"
        #ps.header.stamp = self.get_clock().now().nanoseconds / 1e9

        self.pose_publisher.publish(ps)   


def main(args=None):

    print(args)

    rclpy.init(args=args)
    model = KinematicModel()

    rclpy.spin(model)

    rclpy.shutdown()
