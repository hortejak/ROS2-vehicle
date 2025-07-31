#usr/bin/env python3
import rclpy
from rclpy.node import Node

from interfaces.msg import KinematicInput, KinematicState

import numpy as np
import time

class PID():
    def __init__(self,P=1,I=0,D=0,max_output=100,min_output=-100,N=10):

        # integral part

        self.clamp = False
        self.integral = 0

        # derivative part

        self.derivative_integral = 0
        self.derivative_filter_N = N

        # general

        self.prev_time = 0
        self.max_output = max_output
        self.min_output = min_output

        self.P = P
        self.I = I
        self.D = D
        

    def calculate(self,value):
        dt = self.get_dt()

        P_gain = self.P * value
        I_gain = self.I * self.manage_integral(value,dt)         
        D_gain = self.D * self.manage_derivative(value,dt) if dt != 0 else 0
            

        out_raw = P_gain + I_gain + D_gain
        out = np.clip(a=out_raw,a_min=self.min_output,a_max=self.max_output)

        # integral windup managed by clamping
        self.clamp = True if out != out_raw else False

        return P_gain + I_gain + D_gain
    
    def manage_integral(self,addition,dt):

        if self.clamp:
            return self.integral

        self.integral += addition * dt

        return self.integral

    def manage_derivative(self,addition,dt):

        derivative = (addition - self.derivative_integral) * self.derivative_filter_N
        self.derivative_integral += derivative

        return derivative
    
    def get_dt(self):
        
        if self.prev_time == 0:
            return 0
        
        t = time.time()

        return t - self.prev_time



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