#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from interfaces.msg import KinematicState, KinematicInput
from geometry_msgs.msg import Pose, Point, Quaternion

class KinematicModel(Node):

    # origin point is REAR AXLE

    def __init__(self):
        super().__init__("kinematic_model_py")
        self.get_logger().info("Kinematic model starting")
        self.create_timer(0.02,self.run)

        self.state_publisher = self.create_publisher(KinematicState,'/kinematic/state',10)
        self.pose_publisher = self.create_publisher(Pose,"/odometry/position",10)

        self.input_subscriber = self.create_subscription(KinematicInput,"/kinematic/input",self.get_input,10)

        self.t0 = self.get_clock().now()

        self.length = 2.8
        self.X = np.zeros(4)
        self.u = np.zeros(2)

    def get_input(self, msg):

        self.u[0] = msg.a
        self.u[1] = msg.delta

    def compute_derivatives(self):
        """
        X ..... [x; y; theta; v]

        u ..... [a; delta]

        dX ..... [dx; dy; dtheta; dv] 
        """

        dX = np.zeros(4)
        
        dX[0] = self.X[3] * np.cos(self.X[2])
        dX[1] = self.X[3] * np.sin(self.X[2])
        dX[2] = self.X[3] * np.tan(self.u[1]) / self.length
        dX[3] = self.u[0]

        return dX
    
    def euler2quaternion2D(self,theta):

        q = np.zeros(4)

        q[0] = np.cos(theta/2)
        q[3] = np.sin(theta/2)

        return q
    
    def run(self):

        # get time
        dt = (self.get_clock().now() - self.t0).nanoseconds / 1e9
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

        self.pose_publisher.publish(pose)   


def main(args=None):
    rclpy.init(args=args)
    model = KinematicModel()

    rclpy.spin(model)

    rclpy.shutdown()
