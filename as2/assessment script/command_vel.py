#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities


class CommandVelocity():
    """Driving my robot
    """

    def __init__(self):
        rospy.loginfo("Starting node")
        self.pub = rospy.Publisher("/turtlebot_2/cmd_vel", Twist) # Creating a publisher

    # A function to send velocities until the node is killed
    def send_velocities(self):
        r = rospy.Rate(10) # Setting a rate (hz) at which to publish
        while not rospy.is_shutdown(): # Runnin until killed
            rospy.loginfo("Sending commands")
             # Creating a new message to send to the robot
            
#            twist_msg.linear.x = 0.5
#            twist_msg.angular.z = 1.0
            for i in range (30):
                twist_msg = Twist()
                twist_msg.linear.x = 0.5
                self.pub.publish(twist_msg)
                r.sleep()
            for i in range (30):
                twist_msg = Twist()
                twist_msg.angular.z = 1.57
                self.pub.publish(twist_msg)
                r.sleep()
             # Sending the message via our publisher
             # Calling sleep to ensure the rate we set above

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
    cv.send_velocities() # Calling the function
    rospy.spin()
