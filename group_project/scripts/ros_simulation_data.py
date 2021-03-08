#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 15:18:34 2020

@author: lucamora
"""


import rospy 
import time
from geometry_msgs.msg import Twist,PoseWithCovariance, Quaternion, Point, Pose, Vector3, Vector3Stamped, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Header
from std_msgs.msg import Empty 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
#from keras.layers.core import Dense, Dropout, Activation, Flatten
import matplotlib.pyplot as plt
import numpy as np
import time
import random
import math
import pdb
from cv_bridge import CvBridge, CvBridgeError
import cv2

def takeEnvObservations(): #Function which takes information from the environment in gazebo 
       
        #Take pose information 
        poseData = None
        while poseData is None :
            try:
                poseData = rospy.wait_for_message('/ground_truth/state', Odometry, timeout = 1)
            except:
                rospy.loginfo('Unable to reach the drone Pose topic. Try to connect again')
                
        
        # imuData = None 
        # while imuData is None :
        #     try:
        #         imuData = rospy.wait_for_message('/ardrone/imu', Imu, timeout = 1)
        #     except:
        #         rospy.loginfo('Unable to reach the drone Imu topic. Try to connect again')
        
        # velData = None
        # while velData is None:
        #   try:
        #       velData = rospy.wait_for_message('/fix_velocity', Vector3Stamped, timeout=1)
        #   except:
        #       rospy.loginfo("Unable to reach the drone Imu topic. Try to connect again")
        # altitudeVelDrone = None
       
        # while altitudeVelDrone is None:
        #   try:
        #       altitudeVelDrone = rospy.wait_for_message('/drone/cmd_vel', Twist, timeout=5)
                
        #   except:
        #       rospy.loginfo("Unable to reach the drone velocity topic. Try to connect again")      
              
    
        # return poseData, imuData, velData, altitudeVelDrone
        return poseData

def take_drone_camera_frame():
    camera_frame = None
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/gimbal_bottom_camera/image_raw', Image, timeout = 1)
            # camera_frame= rospy.Subscriber('/gimbal_bottom_camera/image_raw', Image)
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')     
    return cv_image


def take_drone_camera_RGB_frame():
    camera_frame = None
    while camera_frame is None :
        try:
             camera_frame = rospy.wait_for_message('/gimbal_upward_RGB_camera/image_raw', Image, timeout = 1)
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='bgr8')
        #cv2.imshow('RGBframe', cv_image)
        #cv2.waitKey(2)

    return cv_image

def receive_estimated_control_point_P1():
    Point_1_B = None 
    count_for_exit_while = 1
    while Point_1_B is None :
       
        try:
            Point_1_B = rospy.wait_for_message('/P1_estimated_control_point', Point, timeout = 1)
        except:
            rospy.loginfo('Unable to reach the drone Estimated Point P1 topic. Try to connect again')
            if count_for_exit_while > 2:
                
                break
            count_for_exit_while = count_for_exit_while +1
        
    
    return Point_1_B

def receive_estimated_control_point_P2():
    Point_2_B = None 
    count_for_exit_while = 1
    while Point_2_B is None :
         
        try:
            Point_2_B = rospy.wait_for_message('/P2_estimated_control_point', Point, timeout = 1)
        except:
            rospy.loginfo('Unable to reach the drone Estimated Point P2 topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return Point_2_B

            
            
            
def publish_navigation_Thermo_point(x1, y1, x2, y2):
    pub = rospy.Publisher('Thermo_control_point_1', Point, queue_size=1)
    pub2 = rospy.Publisher('Thermo_control_point_2', Point, queue_size=1)
    
   
    #Punto 1
    msg = Point()
    msg.x= x1
    msg.y = y1
    msg.z = 0.0
    pub.publish(msg)
  
    #Punto 2
    msg1 = Point()
    msg1.x= x2
    msg1.y = y2
    msg1.z = 0.0
    pub2.publish(msg1)
  
def  publish_navigation_Thermo_point2(x1, y1, x2, y2):
    pub = rospy.Publisher('Thermo_control_point2_1', Point, queue_size=1)
    pub2 = rospy.Publisher('Thermo_control_point2_2', Point, queue_size=1)
   
    #Punto 1
    msg = Point()
    msg.x= x1
    msg.y = y1
    msg.z = 0.0
    pub.publish(msg)
  
    #Punto 2
    msg1 = Point()
    msg1.x= x2
    msg1.y = y2
    msg1.z = 0.0
    pub2.publish(msg1)
    
    
    
    
def publish_navigation_RGB_point(x1_RGB, y1_RGB, x2_RGB, y2_RGB):
    pub_RGB1 = rospy.Publisher('RGB_control_point_1', Point, queue_size=1)
    pub2_RGB2 = rospy.Publisher('RGB_control_point_2', Point, queue_size=1)
   
    #Punto 1
    msg_RGB = Point()
    msg_RGB.x= x1_RGB
    msg_RGB.y = y1_RGB
    msg_RGB.z = 0.0
    pub_RGB1.publish(msg_RGB)
  
    #Punto 2
    msg1_RGB = Point()
    msg1_RGB.x= x2_RGB
    msg1_RGB.y = y2_RGB
    msg1_RGB.z = 0.0
    pub2_RGB2.publish(msg1_RGB)
    
    # Test
    #rospy.loginfo("msg1_RGB.x: %f", msg1_RGB.x)
    #rospy.loginfo("msg1_RGB.y: %f", msg1_RGB.y)
    
    
    
#Publish Output Image 
def publish_output_image(clustered_image):
    Camera_elaborated_frame_pub = rospy.Publisher('camera_vision_output', Image, queue_size=1)
   # msg = Image()
    msg_frame = CvBridge().cv2_to_imgmsg(clustered_image,"bgr8")
    #msg.data = msg_frame
    #Publish The image 
    Camera_elaborated_frame_pub.publish(msg_frame)
    
def publish_output_RGB_image(clustered_RGB_image):
    camera_elaborated_frame_pub = rospy.Publisher('camera_vision_RGB_output', Image, queue_size=1)
   # msg = Image()
    msg_frame_RGB = CvBridge().cv2_to_imgmsg(clustered_RGB_image,"rgb8")
    #msg.data = msg_frame
    #Publish The image 
    camera_elaborated_frame_pub.publish(msg_frame_RGB)