#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
                                          Image, self.callback)#"/camera/rgb/image_color"
        
#        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
#                                         Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        
#----------------------------------------------------------------------------\/
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #get image
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((65, 120, 0)),#
                                 numpy.array((100, 170, 255)))
                               
#       hsv_thresh = cv2.medianBlur( hsv_thresh, 3)
        kernel = numpy.ones((2,2),numpy.uint8) #make structure
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_OPEN, kernel)# Erosion then Dilation
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_CLOSE, kernel)# Dilation then Erosion

        print numpy.mean(hsv_img[:, :, 0])
        print numpy.mean(hsv_img[:, :, 1])
        print numpy.mean(hsv_img[:, :, 2])
        ret,bitimage = cv2.threshold(hsv_img,0,255,cv2.THRESH_BINARY)
        
        
#==========================================================================
        
#====================================================================================
      
                

        print '===='
        cv2.imshow("Image window", hsv_thresh)
#        cv2.imshow("Image window", bgr_thresh)

image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
