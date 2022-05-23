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

class YawController:

    def __init__(self):

        self.sensor_sub = rospy.Subscriber("mavros/imu/data",Int16,self.sensor_callback)
        self.reset_sub = rospy.Subscriber("controllers/reset", Empty, self.reset_callback)
        self.desired_val_sub = rospy.Subscriber("controller/yaw/desired", Float64, self.desired_val_callback)
        self.pub = rospy.Publisher('controller/yaw/effort', Float64, queue_size=10)
        
        self.init = False

        self.startup_yaw = 0.0

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
        self.kp = rospy.get_param('controller/yaw/kp', 0.0)
        self.ki = rospy.get_param('controller/yaw/ki', 0.0)
        self.kd = rospy.get_param('controller/yaw/kd', 0.0)

    def reset_callback(self, data):
        self.init = True

    def sensor_callback(self,data):
        orientation = data.orientation
        angular_velocity = data.angular_velocity

        # extraction of yaw angle
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        angle_yaw = euler[2]

        if (self.init):
            # at 1st execution, init
            self.startup_yaw = angle_yaw
            self.init = False

        yaw = (angle_yaw - self.startup_yaw) * 180/math.pi
        r = angular_velocity.z * 180/math.pi

        control_effort = self.controller.control(self.desired_val, yaw, r)
        self.pub.publish(Float64(control_effort))

def main(args):
  rospy.init_node('yaw_controller_node')
  controller = YawController()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
