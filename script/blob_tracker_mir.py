#!/usr/bin/env python
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist


###---- Visual Tracking and Servoing----
from sensor_msgs.msg import CompressedImage
import cv2
from Point2D import Point2D


import time
import sys
import argparse

# ---------- Global Variables ---------------
global enable_depth
global enable_vs
global init_p0
global depth_p0
global depth_wrt_startup
global n_points
global reset_desired_points
global reset_previous_points
global desired_points
global previous_points
global flag_alert 

n_points = 8
reset_desired_points = True
reset_previous_points = True
desired_points = []
flag_alert = False

#camera parameters
u0 = 341
v0 = 258
lx = 455
ly = 455
kud =0.00683 
kdu = -0.01424 


def order_point(previous_pts, current_pts):
    
    if(current_pts.shape[0]>0 and current_pts.shape[0]==previous_pts.shape[0]):
        ordered_pts = []
        for ppt in previous_pts:
            index = np.argmin(np.linalg.norm(current_pts - ppt, axis=1)) 
            ordered_pts.append(current_pts[index])
    else :
        ordered_pts = previous_pts
        
    return ordered_pts


def convert2meter(pt,u0,v0,lx,ly):
    return (pt[0]-u0)/lx, (pt[1]-v0)/ly
    


global desired_points2D
#defined desired points 
desired_points2D = [Point2D(-0.243369710525,-0.418902983183),
                  Point2D(-0.0143862444929,-0.429675544624),
                  Point2D(-0.096243540171,-0.352463371199),
                  Point2D(-0.237090345635,-0.291995145385),
                  Point2D(-0.156886982784,-0.189359449174),
                  Point2D(-0.00989097068426,-0.148452126201),
                  Point2D(-0.00165708972955,-0.0341907266445),
                  Point2D(-0.224742039631,-0.0227008870907)]
    
vcam_vs = np.array([0,0,0,0,0,0])
lambda_vs = 0.5


set_mode = [0]*3
Vmax_mot = 1900
Vmin_mot = 1100

angle_wrt_startup = [0]*3
depth_wrt_startup = 0
depth_p0 = 0
rho = 1000.0
gravity = 9.80665

#Conditions
init_a0 = True
init_p0 = True
arming = False
custom_PI = False

set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

enable_depth = False 
enable_vs = 0   # set to tracker status
enable_ping = True 
pinger_confidence = 0
pinger_distance = 0

# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 



def overlay_points(image,points,r,g,b,scale =0.5,offsetx=5, offsety=5):
    index=1
    for pt in points:
        #display overlay
        cv2.circle(image,(int(pt[0]),int(pt[1])),
                   int(4*scale+1), (b,g,r),-1)
        position = (int(pt[0])+offsetx,int(pt[1])+offsety)
        text = "{0}" .format(index)
        cv2.putText(image,text,position,
                    cv2.FONT_HERSHEY_SIMPLEX, scale,(b, g, r, 255),1)
        index+=1
    


def cameracallback(image_data):
    
    
    global n_points
    global reset_desired_points
    global desired_points
    global flag_alert

    
    # get image data
    np_arr = np.fromstring(image_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # call cv function to track blobs
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 100
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(image_np)
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(image_np, keypoints, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #display info
    position = (10,30)
    text = "LEFT click on the image : reset current tracked and desired points. " 
    cv2.putText(blobs,text,position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255, 255),1) 
    position = (10,60)
    text = "RIGHT click on the image : reset desired point ONLY."
    cv2.putText(blobs,text,position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255, 255),1)
    
    

    
    
    # build the 2D points
    current_points = []
    global u0,v0,lx,ly
     
    for keypoint in keypoints:
        pt = keypoint.pt[0],keypoint.pt[1]
        current_points.append([pt[0],pt[1]]);


    global reset_previous_points
    global previous_points
    
    # treat specifically first iteration
    if(reset_previous_points):
        previous_points = current_points
        desired_points = current_points
        reset_previous_points = False
        flag_alert = False
        print("previous_points updated")
    
    
    # order_point
    ordered_points = order_point(np.array(previous_points), np.array(current_points))
    
    #overlay_points(blobs,current_points,200,100,100,0.5,5,-5)
    #overlay_points(blobs,previous_points,0,255,255,0.5,-5,0)
    if(flag_alert==False):
        overlay_points(blobs,ordered_points,0,255,0,0.5,5,0)
        overlay_points(blobs,desired_points,255,0,0,0.5,5,0)
        position = (10,410)
        text = "Green points = current tracked points"
        cv2.putText(blobs,text,position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255, 255),1)
        position = (10,430)
        text = "Red points = desired points"
        cv2.putText(blobs,text,position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255, 255),1)

    if( np.linalg.norm(np.array(previous_points)-np.array(ordered_points)) > 80):
        flag_alert = True
        
    if (flag_alert == True ):
        #print "Alert ! Tracking failed ! Left click to reset tracking points"
        position = (10,410) 
        text = "Alert ! Tracking failed ! Left click to reset tracking points" 
        cv2.putText(blobs,text,position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 0, 255, 255),1)
        overlay_points(blobs,ordered_points,100,100,100,0.5,5,0)
        overlay_points(blobs,desired_points,100,100,100,0.5,5,0)
        
        
    cv2.namedWindow("image")
    # cv2 mouse
    cv2.setMouseCallback("image", click_detect)
    cv2.imshow("image", blobs)
    
    
    #update the previous points
    #print(ordered_points)
    previous_points = ordered_points
    
    #publish points
    ordered_points_reshaped = np.array(ordered_points).reshape(-1)
    current_point_msg = Float64MultiArray(data = ordered_points_reshaped)
    

    #rospy.loginfo(current_point_msg)

    if(np.shape(ordered_points)[0] == n_points and flag_alert==False):
       # print "publish points"
        if(reset_desired_points) : 
            desired_points = ordered_points
            reset_desired_points = False
            print("desired_points updated")
        pub_tracked_point.publish(current_point_msg)
        desired_points_reshaped = np.array(desired_points).reshape(-1)
        desired_points_msg = Float64MultiArray(data = desired_points_reshaped)
        pub_desired_point.publish(desired_points_msg)
        

    
    cv2.waitKey(2)

def click_detect(event,x, y, flags, param):
    global reset_previous_points
    global reset_desired_points
    
    if event == cv2.EVENT_RBUTTONDOWN:
        reset_desired_points = True
        print("desired_points to update")
    
    if event == cv2.EVENT_LBUTTONDOWN:
        reset_previous_points = True
        print("previous_points to update")



def subscriber():
    #camera
    rospy.Subscriber("/br5/usb_cam/image_raw/compressed", CompressedImage, cameracallback,  queue_size = 1)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':

    rospy.init_node('blob_tracker_mir', anonymous=False)  
    
    print('tracker launched')
    
    pub_tracked_point = rospy.Publisher("tracked_points",Float64MultiArray,queue_size=1,tcp_nodelay = True)
    pub_desired_point = rospy.Publisher("desired_points",Float64MultiArray,queue_size=1,tcp_nodelay = True)
    subscriber()


