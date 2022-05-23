#!/usr/bin/env python
import rospy
from script.pid import PID
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from waterlinked_a50_ros_driver.msg import DVL
from alpha_beta_gamma_filter import alpha_beta_gamma_filter
import tf
import math

from PI_Controller import*

class YawController:

    def __init__(self):

        self.sensor_sub = rospy.Subscriber("/dvl/data", DVL, self.sensor_callback)
        self.reset_sub = rospy.Subscriber("controllers/reset", Empty, self.reset_callback)
        self.desired_val_sub = rospy.Subscriber("controller/sway/desired", Float64, self.desired_val_callback)
        self.pub = rospy.Publisher('controller/sway/effort', Float64, queue_size=10)
        
        self.init = False

        self.startup_sway = 0.0

        self.step = 0.02

        self.controller = PID()

        self.v_e0 = 0
        self.vd_e0 = 0

        self.get_params()
        self.set_controller()

    def set_controller(self):
        self.controller.set_gains(self.kp, self.ki, self.kd)
        self.controller.set_step(self.step)
        
    def desired_val_callback(self, msg):
        self.desired_val = msg.data

    def get_params(self):
        self.kp = rospy.get_param('controller/sway/kp', 0.0)
        self.ki = rospy.get_param('controller/sway/ki', 0.0)
        self.kd = rospy.get_param('controller/sway/kd', 0.0)

    def reset_callback(self, data):
        self.v_e0 = 0
        self.vd_e0 = 0
        self.init = True

    def sensor_callback(self,data): 
        v = data.velocity.y  # Linear velocity along y (sway) 
        alpha = 0.45
        beta = 0.1

        # Filter : 
        vd_e, v_e = alpha_beta_gamma_filter(self.v_e0, self.vd_e0, 0, v, alpha, beta, 0.1) 
        self.v_e0, self.vd_e0 = v_e, vd_e

        control_effort = self.controller.control(self.desired_val, v_e, vd_e)
        self.pub.publish(Float64(control_effort))

def main(args):
  rospy.init_node('sway_controller_node')
  controller = YawController()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
