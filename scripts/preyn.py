#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                  # The ROS python bindings
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from sensor_msgs.msg import LaserScan  # The message type of the laser scan

from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image
import time

class Braitenberg():

    def __init__(self, name):

        #rospy.loginfo("Starting node %s" % name)
        self.bridge = CvBridge()
        self.timer = time.time()
        self.checkstat = True
        self.turn =  True
        self.issafe = True       #- spin around at loaction till box seen
        self.turnsmallest = 0.0        
        self.iswall = False     #- if see on left : turn 140 deg and run away else :turn -140 deg, set run away to true
        self.isrun = False       #-start timer, if running away for X seconds stop rotate 180          # Creatingan OpenCV bridge object used to create an OpenCV image from the ROS image
        cv2.namedWindow("Image window", 1)  # Opening a window to show the image
        cv2.startWindowThread()

        #"/camera/rgb/image_color"
        #"/turtlebot_1/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
            "/camera/rgb/image_color", # The topic to which it should listened
            #"/turtlebot_2/camera/rgb/image_raw",
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        self.laser_sub = rospy.Subscriber(  # Creating a subscriber listening to the laser scans
            '/scan',                        # The topic to which it should listend
            #'/turtlebot_2/scan',
            LaserScan,                      # The data type of the topic
            callback=self.laser_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            "/cmd_vel",                     # The topic to which it should publish
            #"/turtlebot_2/cmd_vel",
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
        )
        
        
    def image_callback(self, img):
        #rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something
        rate = rospy.Rate(10)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image 
        except CvBridgeError, e:
            print e
        #chance the below colour thing to the hvs 
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #get image
        
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((65, 120, 0)),#0,220,0
                                 numpy.array((100, 170, 255)))# get  from range-150,255,255

#        hsv_thresh = cv2.inRange(hsv_img,
#                                 numpy.array((0, 220, 0)),
#                                 numpy.array((150, 255, 255)))
                               
        hsv_thresh = cv2.medianBlur( hsv_thresh, 3)#medium filter
        kernel = numpy.ones((2,2),numpy.uint8) #make structure
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_OPEN, kernel)# Erosion then Dilation
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_CLOSE, kernel)# Dilation then Erosion
        hsv_thresh1 = hsv_thresh
        cv2.imshow("Image window", hsv_thresh)    
        ret,hsv_thresh1 = cv2.threshold(hsv_thresh,0,255,cv2.THRESH_BINARY)  
        
        hsv_thresh/=255 # divides whole image by 225
        twist_msg = Twist()
       # print "====="
        #mean_leftim = numpy.mean(hsv_thresh[0:320, 0:240])
        #mean_rightim = numpy.mean(hsv_thresh[320:640, 240:480])
       
        wholeintensity = numpy.mean(hsv_thresh)
        leftim = numpy.mean(hsv_thresh[:, 0:320])/wholeintensity
        rightim = numpy.mean(hsv_thresh[:, 320:640])/wholeintensity
        
        wholeintensity1 = numpy.sum(hsv_thresh) 
        
        if self.isrun:
            print'running'
            if wholeintensity1 > 150:
                self.turn = True
            else:
                self.turn = False
        else:
            if wholeintensity1 > 150: #200# if pred seen and not runing = change flag
                print 'robot seen' 
                self.issafe = False
                self.timer = self.timer + 40
            else:
                print 'all clear' 
                self.issafe = True
            
        if not self.issafe:
            if self.isrun:
                if not self.iswall:
                    twist_msg.linear.x = 0.3
                    if self.turn and leftim > rightim:
                        twist_msg.angular.z = (numpy.pi/2)*1 
                    elif self.turn and leftim < rightim:
                        twist_msg.angular.z = numpy.pi/2    
                    else:
                        twist_msg.angular.z = 0.0 
                elif self.iswall:
                    twist_msg.linear.x = -0.06
                    twist_msg.angular.z = self.turnsmallest
            else:
                twist_msg.linear.x = 0
                twist_msg.angular.z = numpy.pi #neg 120deg
                self.cmd_vel_pub.publish(twist_msg)
                rate.sleep()
                self.isrun = True
        else:
            twist_msg.angular.z = numpy.pi/2
            self.isrun = False
            
        if self.timer >= time.time():
            print'in time'
        else:
            self.isrun = False
            print self.timer
            print time.time()
            
                
        #######################################################################


        cv2.imshow("Image window", hsv_thresh)  # Showing the image
        self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message
        rate.sleep()                         # Sleep to ensure rate of 10hz 
        
    def laser_callback(self, msg):
        ranges = msg.ranges                 # Getting the range values of the laser rays
        leftside = numpy.mean(numpy.nanmin(ranges[0:320]))
        rightside = numpy.mean(numpy.nanmin(ranges[321:640]))
#        
#        print leftside
#        print '---'
#        print rightside
     
        if leftside < rightside:
            self.turnsmallest = numpy.pi/4
           # print 'left'
        elif leftside > rightside:
            self.turnsmallest = (numpy.pi/4)*-1
        else:
            self.turnsmallest = numpy.pi           # print 'turn right'
        
        min_distance = numpy.nanmin(ranges)  # Using numpy's nanmin function to find the minimal value and ignore nan values
        if min_distance < 0.95:                    # If the robot is close, then
            self.iswall = True 
        else:
            self.iswall = False
             
   #note to self, dnt need to make it binardy, just divide
   # plan, hav this meth hand the seeing of the robot, the rotoing of it self but let laser hand the moving" runway"

# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("braitenberg")     # Create a node of name braitenberg
    b = Braitenberg(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
