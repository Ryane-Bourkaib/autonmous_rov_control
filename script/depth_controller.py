#!/usr/bin/env python
import rospy
from script.pid import PID
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import tf
import math

from PI_Controller import*

class DepthController:

    def __init__(self):

        self.sensor_sub = rospy.Subscriber("mavros/imu/data",Int16,self.sensor_callback)
        self.reset_sub = rospy.Subscriber("controllers/reset", Empty, self.reset_callback)
        self.desired_val_sub = rospy.Subscriber("controllers/depth/desired", Float64, self.desired_val_callback)
        self.pub = rospy.Publisher('controller/depth/effort', Float64, queue_size=10)
        
        self.init = False
        self.depth_wrt_startup = 0

        self.rho = 1000.0
        self.gravity = 9.80665
        self.step = 0.02

        self.controller = PID()

        self.get_params()
        self.set_controller()

    def set_controller(self):
        self.controller.set_gains(self.kp, self.ki, self.kd)
        self.controller.set_step(self.step)
        
    def desired_val_callback(self, msg):
        self.desired_val = msg.data

    def get_params(self):
        self.kp = rospy.get_param('kp', 0.0)
        self.ki = rospy.get_param('ki', 0.0)
        self.kd = rospy.get_param('kd', 0.0)

    def reset_callback(self, data):
        self.init = True

    def sensor_callback(self, data):
        pressure = data.fluid_pressure

        if (self.init):
            # 1st execution, init
            depth_p0 = (pressure - 101300)/(self.rho * self.gravity)
            self.init = False

        self.depth_wrt_startup = (pressure - 101300)/(self.rho * self.gravity) - depth_p0
        
        control_effort = self.controller.control(
            self.desired_val, self.depth_wrt_startup, r)
        self.pub.publish(Float64(control_effort))

def main(args):
  rospy.init_node('depth_controller_node')
  controller = DepthController()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
