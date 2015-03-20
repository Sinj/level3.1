#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                  # The ROS python bindings
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from sensor_msgs.msg import LaserScan  # The message type of the laser scan
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image

imageAnalysisComplete = False
laserAnalysisComplete = False

listOfPossibleActions = [];


class WeightedAction():
        
    def __init__(self, speed, angle, weight):
        self.forward_speed = speed
        self.turn_angle = angle
        self.weighting = weight

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

        rospy.loginfo("Subscribing and Publishing")

        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
            "/camera/rgb/image_raw",      # The topic to which it should listened
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            "/cmd_vel",                     # The topic to which it should publish
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
        )
        
        self.laser_sub = rospy.Subscriber(  # Creating a subscriber listening to the laser scans
           "/scan",                          # The topic to which it should listend
           LaserScan,                      # The data type of the top
           callback=self.laser_callback,   # The callback function that is triggered when a new message arrives
           queue_size=1                    # Disregard every message but the latest
        )  

    def send_twist_msg(self):
        global imageAnalysisComplete
        global laserAnalysisComplete
        global listOfPossibleActions
        
        twist_msg = Twist();
        forward_speed = 0;
        turn_angle = 0;
        
        highest_weighting = 0;
        
        if imageAnalysisComplete and laserAnalysisComplete:
            for actionIndex in range(len(listOfPossibleActions)):
                if listOfPossibleActions[actionIndex].weighting > highest_weighting:
                    forward_speed = listOfPossibleActions[actionIndex].forward_speed
                    turn_angle = listOfPossibleActions[actionIndex].turn_angle
                    highest_weighting = listOfPossibleActions[actionIndex].weighting
              
            twist_msg.linear.x = forward_speed                # Set linear speed
            twist_msg.angular.z = turn_angle       # Set angular speed
            
            self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message
            
            del listOfPossibleActions [:]
            imageAnalysisComplete = False;
            laserAnalysisComplete = False;
            
            
        
    def image_callback(self, img):
   
        
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        green_filter = cv2.inRange(cv_image,numpy.array((0, 225, 0)),numpy.array((255, 255, 255)))
        
        hsv_filter = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hue_filter = cv2.inRange(hsv_filter,numpy.array((80, 0, 0)),numpy.array((90, 255, 255)))
        
        kernel = numpy.ones((5,5), numpy.uint8)
        eroded_filter = cv2.erode(hue_filter,kernel,iterations = 1)
                            
        #gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to gray scale

        mid_width = img.width / 2;

        left_image = eroded_filter[:, 0:mid_width]
        right_image = eroded_filter[:, mid_width:img.width]
        
        mean_intensity = numpy.mean(eroded_filter)        
        left_mean_intensity = numpy.mean(left_image)
        right_mean_intensity = numpy.mean(right_image)   
        
        normalised_left_intensity = left_mean_intensity / 255
        normalised_right_intensity = right_mean_intensity / 255
                   
        turn_angle = 0
        forward_speed = 0.1
        
        if mean_intensity == 0.0: #nothing green can be seen
            action_weight = 0
        else:
            action_weight = 5
                
        
        if normalised_left_intensity > normalised_right_intensity:
            turn_angle = 0.2#rotate left
        else:
            turn_angle = -0.2 #rotate right
               
        print "====="
        mean_intensity = numpy.mean(green_filter)           # Getting the mean intensity of the whole image
        normalised_mean_intensity = mean_intensity / 255  # Normalising the intensity
        print "Mean intensity: ", mean_intensity
        print "Normalised mean intensity: ", normalised_mean_intensity

        #cv2.imshow("Image window", green_filter)  # Showing the image
        cv2.imshow("Image window", eroded_filter)  # Showing the image
        
        global listOfPossibleActions
        listOfPossibleActions.append(WeightedAction(forward_speed, turn_angle, action_weight))
        
        global imageAnalysisComplete
        imageAnalysisComplete = True
        
        self.send_twist_msg()
             
        
    # This function helps you to emulate e Braitenberg vehicle by allowing you
    # to virtually send velocities to the separate wheels. There is no need to
    # change this function at any point. Please refer to the example and the
    # task description on how to use it
    def wheel_motor_power_to_twist_msg(self, left_wheel_power, right_wheel_power=None):
        """Emulating a differential wheel drive where you can set the power for
        each wheel separately. Takes wheel powers between -1.0 and 1.0
        Used for Braitenberg vehicle type 2. If only left_wheel_power is set,
        the right wheel will have the same power to emulate a vehicle type 1.

        :param left_wheel_power: Power to the left wheel 1.0 fast forwards, -1.0 fast backwards
        :param right_wheel_power: Power to the right wheel 1.0 fast forwards, -1.0 fast backwards
        """

        # for Braitenberg vehicle type 1. Only setting left_wheel_power results
        # in both wheels receiving the same power
        if right_wheel_power == None:
            right_wheel_power = left_wheel_power

        # Making sure the power is between -1 and 1
        if left_wheel_power > 1.0 or left_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return
        if right_wheel_power > 1.0 or right_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return

        # Calculating linear and angular speed using fixed values. Results in a
        # top speed of up to 0.2 m/s
        left_wheel = [0.1*left_wheel_power, -1.0*left_wheel_power]
        right_wheel = [0.1*right_wheel_power, 1.0*right_wheel_power]

        # Generating and publishing the twist message
        twist = Twist()
        twist.linear.x = left_wheel[0] + right_wheel[0]
        twist.angular.z = left_wheel[1] + right_wheel[1]
        return twist

    def laser_callback(self, msg):

        mid_range_point = len(msg.ranges) / 2;
             
        left_ranges = msg.ranges[0:mid_range_point]                 # Getting the range values of the laser rays
        right_ranges = msg.ranges[mid_range_point:len(msg.ranges)]
                
        left_min_distance = numpy.nanmin(left_ranges)  # Using numpy's nanmin function to find the minimal value and ignore nan values               
        right_min_distance = numpy.nanmin(right_ranges)
        
        forward_speed = 0.0
        turn_angle = 0.0
        action_weight = 2
        
        if left_min_distance < right_min_distance:
            min_distance = left_min_distance
            turn_angle = 0.5 # rotate right
        else:
            min_distance = right_min_distance
            turn_angle = 0.5 # rotate left   
        
        if min_distance > 2:
            turn_angle = 0.0
            forward_speed = 0.1
        
        if min_distance < 0.9:
            action_weight = 10
            
        if min_distance < 1.5:
            action_weight = 7
            
        if min_distance <= 2:
            action_weight = 4
               
        global listOfPossibleActions
        listOfPossibleActions.append(WeightedAction(forward_speed, turn_angle, action_weight))        
        
        global laserAnalysisComplete
        laserAnalysisComplete = True;
        
        self.send_twist_msg()


# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("braitenberg")     # Create a node of name braitenberg
    b = Braitenberg(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
