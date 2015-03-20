#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                  # The ROS python bindings
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from sensor_msgs.msg import LaserScan         # The message type of the laser scan
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image
from random import choice, randint     # Could be used for the random turning of the robot

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

        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
            "/camera/rgb/image_color",      # The topic to which it should listened
            #"/turtlebot_2/camera/rgb/image_raw",
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        
#        self.laser_sub = rospy.Subscriber(  # Creating a subscriber listening to the laser scans
#            '/scan',
#            #'/turtlebot_2/scan',                        # The topic to which it should listend
#            LaserScan,                      # The data type of the topic
#            callback=self.laser_callback,   # The callback function that is triggered when a new message arrives
#            queue_size=1                    # Disregard every message but the latest
#        )
        
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            "/cmd_vel",                     # The topic to which it should publish
            #"/turtlebot_2/cmd_vel",
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
        )
        

    def image_callback(self, img):
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        #gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to gray scale
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #######################################################################
        # CODE GOES HERE

        twist_msg = Twist()
        print "====="
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((65, 120, 0)),
                                 numpy.array((100, 170, 255)))
        
#        hsv_thresh = cv2.inRange(hsv_img,
#                                 numpy.array((0, 220, 0)),
#                                 numpy.array((150, 255, 255)))
                                 
#        hsv_thresh = cv2.medianBlur( hsv_thresh, 3)
        kernel = numpy.ones((2,2),numpy.uint8)
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_OPEN, kernel)
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_CLOSE, kernel)
        
        ret, bw_img = cv2.threshold(hsv_thresh,0,255,cv2.THRESH_BINARY)
        # change image to 0 & 1           

        #get mean of left half of image 
        bw_img/= 255
        sum_leftim = numpy.sum(bw_img[:, 0:215])
        sum_midim = numpy.sum(bw_img[:, 215:425])
        sum_rightim = numpy.sum(bw_img[:, 425:640])
        nsum_leftim = sum_leftim/numpy.sum (bw_img)
        nsum_midim = sum_midim/numpy.sum (bw_img)
        nsum_rightim = sum_rightim/numpy.sum (bw_img)
        #get mean of right half of image        
        
                           
        mean_intensity = numpy.sum(bw_img)           # Getting the mean intensity of the whole image
#        left_normalised_mean_intensity = mean_leftim / 255  # Normalising the intensity
#        right_normalised_mean_intensity = mean_rightim / 255
#        normalised_mean_intensity = mean_intensity / 255
        print "Mean intensity left: ", sum_leftim
        print "Mean intensity right: ", sum_rightim
        print "Sum intensity middle: ", sum_midim
        print "Normalised mean intensity: ", mean_intensity

        #right_wheel_power = left_normalised_mean_intensity    # Using normalised intensity to determine the speed of the robot
        #left_wheel_power = right_normalised_mean_intensity
        # twist_msg = self.wheel_motor_power_to_twist_msg(wheel_power)  # Creating twist message from power value
        # twist_msg = self.wheel_motor_power_to_twist_msg(left_wheel_power, right_wheel_power)
       # twist_msg.angular.z = 0
        
        if mean_intensity < 180:
            twist_msg.linear.x = 0
            twist_msg.angular.z = numpy.pi/8
        else:
            twist_msg.linear.x = 0.3
            twist_msg.angular.z = (nsum_leftim - nsum_rightim)* numpy.pi/8
            print twist_msg.angular.z
            
        #######################################################################


        cv2.imshow("Image window", hsv_thresh)  # Showing the image
        self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message


    def laser_callback(self, msg):
        """A callback function that is triggered when ever a new message arrives
        on the topic that is defined by the subscriber.

        :param msg: The message received by the subscriber of the type as defined when creating the subscriber
        """
#        robotfunarray = [5,9,1,3,7,5,10,8]
        ranges = msg.ranges                 # Getting the range values of the laser rays
        min_distance = numpy.nanmin(ranges)  # Using numpy's nanmin function to find the minimal value and ignore nan values
        rospy.loginfo("Minimum distance: %f" % min_distance)
        twist_msg = Twist()        # Creating a new Twist type message
        if min_distance < 0.75:                    # If the robot is close, then
            rospy.loginfo("Turn")  # Turn the robot
            rate = rospy.Rate(10)  # Set a rate of 10hz
            now = rospy.Time.now().to_sec()          # Get the current time as seconds.milliseconds
            end_time = now + 1 #choice(robotfunarray)                     # Defining for how long the robot should turn.
#            angle_velocity = randint(-3 ,3)                     # Defining the angular velocity and direction of the turn. Use np.pi to get PI.
            while end_time >= rospy.Time.now().to_sec() and not rospy.is_shutdown():                               # While the run time is not up, do
                twist_msg.linear.x = 0                # Set linear speed
                twist_msg.angular.z = numpy.pi/4
                rospy.loginfo("RUN") # Set angular speed
                self.cmd_vel_pub.publish(twist_msg)  # Publish Twist message
                rate.sleep()
#
#            twist_msg.angular.z = 0                         # Sleep to ensure rate of 10hz
#        else:                                    # If the robot is far away from everything, then
#            rospy.loginfo("Straight")
#            twist_msg.linear.x = 0.2                 # Set linear movement
#            twist_msg.angular.z = 0              # Set angular movement
#            self.cmd_vel_pub.publish(twist_msg)  # Publish Twist message

# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("braitenberg")     # Create a node of name braitenberg
    b = Braitenberg(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
