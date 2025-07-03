#usr/bin/env python3
import rclpy
from rclpy.node import Node

from interfaces.msg import KinematicInput, KinematicState

import numpy as np

class PID():
    def __init__(self,P=1,I=0,D=0):
        self.P = P
        self.I = I
        self.D = D

    def calculate(self,value):

        P_gain = self.P * value
        I_gain = 0
        D_gain = 0

        return P_gain + I_gain + D_gain


class KinematicLongitudinalPID(Node):
    def __init__(self):
        super().__init__("kinematic_longitudinal_PID")

        self.v_ref = 50/3.6

        self.pid = PID(P=1,I=0,D=0)

        dt = 0.02

        self.X = np.zeros(4)
        self.u = np.zeros(2)

        self.create_subscription(KinematicState,"state/kinematic",self.state_cb,10)
        self.input_publisher = self.create_publisher(KinematicInput,"input/kinematic",10)
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