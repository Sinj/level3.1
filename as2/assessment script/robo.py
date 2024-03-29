#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                  # The ROS python bindings
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image


class Braitenberg():
    """A class to make a Braitenberg vehicle
    """

    # __init__ is a built-in python function and needs to start and end with *two* underscores
    def __init__(self, name):
        """Function to initialise the class. Called when creating a new instance

        :param name: The name of the ros node
        """

        rospy.loginfo("Starting node %s" % name)
        self.bridge = CvBridge()            # Creatingan OpenCV bridge object used to create an OpenCV image from the ROS image
        cv2.namedWindow("Image window", 1)  # Opening a window to show the image
        cv2.startWindowThread()

        #"/camera/rgb/image_color"
        #"/turtlebot_1/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
            "/camera/rgb/image_color",      # The topic to which it should listened
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            "/cmd_vel",                     # The topic to which it should publish
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
        )

    def image_callback(self, img):
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image 
        except CvBridgeError, e:
            print e
        #chance the below colour thing to the hvs 
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #get image
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((65, 120, 0)),#0,220,0
                                 numpy.array((100, 170, 255)))# get  from range-150,255,255
                               
        hsv_thresh = cv2.medianBlur( hsv_thresh, 3)#medium filter
        kernel = numpy.ones((2,2),numpy.uint8) #make structure
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_OPEN, kernel)# Erosion then Dilation
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_CLOSE, kernel)# Dilation then Erosion

        print numpy.mean(hsv_img[:, :, 0])
        print numpy.mean(hsv_img[:, :, 1])
        print numpy.mean(hsv_img[:, :, 2])
        #ret,bitimage = cv2.threshold(hsv_img,0,255,cv2.THRESH_BINARY)           
        cv2.imshow("Image window", hsv_thresh)    
      
        ret,bw_imag = cv2.threshold(hsv_thresh,0,255,cv2.THRESH_BINARY)  
       #######################################################################
        # CODE GOES HERE
        bw_imag/=255 # divides hold image by 225
        twist_msg = Twist()
        print "====="
        #mean_leftim = numpy.mean(hsv_thresh[0:320, 0:240])
        #mean_rightim = numpy.mean(hsv_thresh[320:640, 240:480])
        leftim = numpy.sum(bw_imag[:, 0:320]) # can be replaced with .sum or mean
        rightim = numpy.sum(bw_imag[:, 320:640])
        wholeintensity = numpy.sum(bw_imag)           # Getting the mean intensity of the whole image
        left_intensity = leftim / numpy.sum(bw_imag)  # Normalising the intensity
        right_intensity = rightim / numpy.sum(bw_imag)
        #normalised_mean_intensity = mean_intensity / 255
        print "Mean intensity left: ", leftim
        print "Mean intensity right: ", rightim
        print "Normalised mean intensity: ", wholeintensity

        #right_wheel_power = left_normalised_mean_intensity    # Using normalised intensity to determine the speed of the robot
        #left_wheel_power = right_normalised_mean_intensity
        # twist_msg = self.wheel_motor_power_to_twist_msg(wheel_power)  # Creating twist message from power value
        # twist_msg = self.wheel_motor_power_to_twist_msg(left_wheel_power, right_wheel_power)
        
        if wholeintensity < 160:
            twist_msg.linear.x = 0
            twist_msg.angular.z = numpy.pi/4
        else:    
            twist_msg.linear.x =  0.3
            twist_msg.angular.z = (left_intensity - right_intensity)* numpy.pi/4
       
            
        #######################################################################


        cv2.imshow("Image window", hsv_thresh)  # Showing the image
        self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message


   
   

# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("braitenberg")     # Create a node of name braitenberg
    b = Braitenberg(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
