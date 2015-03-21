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
            "/turtlebot_1/camera/rgb/image_raw",      # The topic to which it should listened
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        #/turtlebot_1/cmd_vel
        #/cmd_vel
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            "/turtlebot_1/cmd_vel",                     # The topic to which it should publish
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
        )

    def image_callback(self, img):
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to gray scale

        #######################################################################
        # CODE GOES HERE
        twist_msg = Twist()
        print "====="
        mean_leftim = numpy.mean(gray_image[:, 0:240])#[0:320, 0:240]
        mean_rightim = numpy.mean(gray_image[:, 240:480])#[320:640, 240:480]
        mean_intensity = numpy.mean(gray_image)           # Getting the mean intensity of the whole image
        left_normalised_mean_intensity = mean_leftim / 255  # Normalising the intensity
        right_normalised_mean_intensity = mean_rightim / 255
        normalised_mean_intensity = mean_intensity / 255
        print "Mean intensity left: ", mean_leftim
        print "Mean intensity right: ", mean_rightim
        print "Normalised mean intensity: ", right_normalised_mean_intensity

        
        
        
        twist_msg.linear.x = normalised_mean_intensity * 0.3
        twist_msg.angular.z = (left_normalised_mean_intensity - right_normalised_mean_intensity)* numpy.pi/4
       
            
        #######################################################################


        cv2.imshow("Image window", gray_image)  # Showing the image
        self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message


   
    


# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("braitenberg")     # Create a node of name braitenberg
    b = Braitenberg(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
