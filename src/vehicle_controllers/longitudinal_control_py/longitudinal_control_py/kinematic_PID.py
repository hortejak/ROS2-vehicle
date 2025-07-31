#usr/bin/env python3
import rclpy
from rclpy.node import Node

from interfaces.msg import KinematicInput, KinematicState
from general_controllers_py.PID import PID # type: ignore

import numpy as np

class KinematicLongitudinalPID(Node):
    def __init__(self):
        super().__init__("kinematic_longitudinal_PID")

        self.v_ref = 50/3.6

        self.pid = PID(P=1,I=0,D=0)

        dt = 0.02

        self.X = np.zeros(4)
        self.u = np.zeros(2)

        self.create_subscription(KinematicState,"state/kinematic",self.state_cb,10)
        self.get_logger().info("Subscribing from state/kinematic")
        self.input_publisher = self.create_publisher(KinematicInput,"input/kinematic",10)
        self.get_logger().info("Publishing to input/kinematic")
        self.create_timer(dt,self.run)      
        
    
    def state_cb(self,msg):

        self.X[3] = msg.v

    def run(self):
        
        u = KinematicInput()
        u.a = self.pid.calculate(self.v_ref - self.X[3])

        self.input_publisher.publish(u)




def main(args=None):
    rclpy.init()
    controller = KinematicLongitudinalPID()
    rclpy.spin(controller)
    rclpy.shutdown()