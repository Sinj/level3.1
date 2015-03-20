#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                  # The ROS python bindings
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from sensor_msgs.msg import LaserScan  # The message type of the laser scan
from kobuki_msgs.msg import BumperEvent
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
from random import randint
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image
import time

simulatedTest = False
hidingMode = False
spotted = False

imageAnalysisComplete = False
laserAnalysisComplete = False

listOfPossibleActions = []
noOfStraightActions = 0


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

        imageSubscribePath = "camera/rgb/image_raw"
        laserSubscribePath = "scan"
        bumperSubscribePath = "events/bumper"
        velocityPublishPath = "cmd_vel"
        
        if simulatedTest:
            imageSubscribePath = "turtlebot_1/" + imageSubscribePath
            laserSubscribePath = "turtlebot_1/" + laserSubscribePath
            bumperSubscribePath = "turtlebot_1/turtlebot_1/" + bumperSubscribePath
            velocityPublishPath = "turtlebot_1/" + velocityPublishPath
        else:
            bumperSubscribePath = "mobile_base/" + bumperSubscribePath

        rospy.loginfo("Starting node %s" % name)
        self.bridge = CvBridge()            # Creatingan OpenCV bridge object used to create an OpenCV image from the ROS image
        cv2.namedWindow("Image window", 1)  # Opening a window to show the image
        cv2.startWindowThread()

        rospy.loginfo("Subscribing and Publishing")

        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
            imageSubscribePath,      # The topic to which it should listened
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
                
        self.laser_sub = rospy.Subscriber(  # Creating a subscriber listening to the laser scans
           laserSubscribePath,                          # The topic to which it should listend
           LaserScan,                      # The data type of the top
           callback=self.laser_callback,   # The callback function that is triggered when a new message arrives
           queue_size=1                    # Disregard every message but the latest
        )
        
        self.laser_sub = rospy.Subscriber(  # Creating a subscriber listening to the laser scans
           bumperSubscribePath,                          # The topic to which it should listend
           BumperEvent,                      # The data type of the top
           callback=self.bumper_callback,   # The callback function that is triggered when a new message arrives
           queue_size=1                    # Disregard every message but the latest
        )  
        
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            velocityPublishPath,                     # The topic to which it should publish
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
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
                print "Action ", actionIndex, ": ", listOfPossibleActions[actionIndex].forward_speed, listOfPossibleActions[actionIndex].turn_angle, listOfPossibleActions[actionIndex].weighting
                if listOfPossibleActions[actionIndex].weighting > highest_weighting:
                    forward_speed = listOfPossibleActions[actionIndex].forward_speed
                    turn_angle = listOfPossibleActions[actionIndex].turn_angle
                    highest_weighting = listOfPossibleActions[actionIndex].weighting
             
            if not spotted:
                global noOfStraightActions
                if turn_angle == 0.0:
                     noOfStraightActions += 1
                else:
                     noOfStraightActions = 0
                
                if noOfStraightActions >= 400:
                    randTurn = randint(0,10);
                    print "**sent_twist_msg - Random int: ", randTurn, "**"
                    if randTurn > 5:
                        turn_angle = 0.3
                    else:
                        turn_angle = -0.3
                    forward_speed = 0.0
                    noOfStraightActions = 0
                
            print "Chosen action: ", forward_speed, turn_angle, highest_weighting             
             
            twist_msg.linear.x = forward_speed     # Set linear speed
            twist_msg.angular.z = turn_angle       # Set angular speed
            
            self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message
            
            del listOfPossibleActions [:]
            imageAnalysisComplete = False;
            laserAnalysisComplete = False;
            
    def bumper_callback(self, bump):

        if bump.state == bump.PRESSED:
            stop_twist_msg = Twist();
            stop_twist_msg.linear.x = 0.0            
            self.cmd_vel_pub.publish(stop_twist_msg)
            
            reverse_twist_msg = Twist();
            reverse_twist_msg.linear.x = -0.1
            for x in range(0,20):
                time.sleep(1)
                self.cmd_vel_pub.publish(reverse_twist_msg)
                
            turn_twist_msg = Twist();
            turn_twist_msg.linear.x = -1.0
            if bump.bumper == bump.LEFT:
                turn_twist_msg.angular.z = -0.7
            if bump.bumper == bump.RIGHT:
                turn_twist_msg.angular.z = 0.7
                   
            self.cmd_vel_pub.publish(turn_twist_msg)
        
            print "Bumper event!"
        
    def image_callback(self, img):
   
        
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        hsv_filter = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        global simulatedTest
        if simulatedTest:        
            hue_filter = cv2.inRange(hsv_filter,numpy.array((50, 0, 0)),numpy.array((60, 255, 255)))
        else:
            hue_filter = cv2.inRange(hsv_filter,numpy.array((80, 0, 0)),numpy.array((90, 255, 255)))
            
        kernel = numpy.ones((5,5), numpy.uint8)
        eroded_filter = cv2.erode(hue_filter,kernel,iterations = 1)        
        
        mid_width = img.width / 2

        left_image = eroded_filter[:, 0:mid_width]
        right_image = eroded_filter[:, mid_width:img.width]
                
        mean_intensity = numpy.mean(eroded_filter)        
        left_mean_intensity = numpy.mean(left_image)
        right_mean_intensity = numpy.mean(right_image)
        
        centX = -1
        centY = -1
        M = cv2.moments(eroded_filter)
        if M['m10'] > 0 and M['m00'] > 0 and M['m01'] > 0:
            centX = int(M['m10']/M['m00'])
            centY = int(M['m01']/M['m00'])


        print "Centroid:"
        print centX, ",", centY
        print "##########"
        
        normalised_left_intensity = left_mean_intensity / 255
        normalised_right_intensity = right_mean_intensity / 255
                   
        turn_angle = 0
        forward_speed = 0.1
        
        global hidingMode
        if hidingMode:        
            if mean_intensity == 0.0:
                left_color = cv_image[:, 0:mid_width]
                right_color = cv_image[:,mid_width:img.width]
                normalised_left_intensity = numpy.mean(left_color) / 255
                normalised_right_intensity = numpy.mean(right_color) / 255
            
            if centX > (mid_width-100):
               turn_angle = centX/100000 #rotate left    
               
            if centX < (mid_width+100):
                turn_angle = -(centX/100000) #rotate left    
            
            if normalised_left_intensity < normalised_right_intensity:
               turn_angle = centX/1000 #rotate left
            else:
               turn_angle = -(centX/1000) #rotate right
               
            action_weight = 5
        else:
            global spotted
            if mean_intensity == 0.0: #nothing green can be seen
                action_weight = 0
                spotted = False
            else:
                if centY > -1 and centY <=140: #very close to prey, so unlikely to hit obstacle
                    action_weight = 8             
                else:
                    #possible obstacles in the way
                    action_weight = 5
                
                spotted = True
             
            if normalised_left_intensity > normalised_right_intensity:
                turn_angle = 0.2#rotate left
            else:
                turn_angle = -0.2 #rotate right
               
        print "====="
        mean_intensity = numpy.mean(eroded_filter)           # Getting the mean intensity of the whole image
        normalised_mean_intensity = mean_intensity / 255  # Normalising the intensity
        print "Mean intensity: ", mean_intensity
        print "Normalised mean intensity: ", normalised_mean_intensity

        cv2.imshow("Image window", cv_image)  # Showing the image
        #cv2.imshow("Image window", eroded_filter)  # Showing the image
        
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
        
        max_distance = numpy.nanmax(msg.ranges)
        print "Maximum distance: ", max_distance
        
        forward_speed = 0.0
        turn_angle = 0.0
        action_weight = 2
        
        if left_min_distance < right_min_distance:
            min_distance = left_min_distance
            turn_angle = 0.5 # rotate left
        else:
            min_distance = right_min_distance
            turn_angle = -0.5 # rotate right

        print "Laser min distance:", min_distance
        
        if max_distance < 4 and min_distance > 2:
            randTurn = randint(0,10);
            print "**laser_callback - Random int: ", randTurn, "**"
            if randTurn > 5:
                turn_angle = 0.4
            else:
                turn_angle = -0.4
            action_weight = 2
        else:
            if min_distance < 0.9:
                action_weight = 10
                
            if min_distance < 1:
                action_weight = 7
                
            if min_distance <= 2:
                action_weight = 4
               
        global listOfPossibleActions
        print "Laser forward speed:", forward_speed
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
