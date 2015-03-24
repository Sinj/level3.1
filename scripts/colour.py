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
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",
                                          Image, self.callback)
        
#        self.image_sub = rospy.Subscriber("/turtlebot_2/camera/rgb/image_raw",
#                                         Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

#        bgr_thresh = cv2.inRange(cv_image,
#                                 numpy.array((200, 150, 150)),
#                                 numpy.array((255, 255, 255)))
#----------------------------------------------------------------------------\/
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((65, 120, 0)),
                                 numpy.array((100, 170, 255)))
        
#        hsv_thresh = cv2.inRange(hsv_img,
#                            numpy.array((0, 220, 0)),
#                            numpy.array((150, 255, 255)))
                                 
#        hsv_thresh = cv2.medianBlur( hsv_thresh, 3)
        kernel = numpy.ones((2,2),numpy.uint8)
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_OPEN, kernel)
        hsv_thresh = cv2.morphologyEx(hsv_thresh, cv2.MORPH_CLOSE, kernel)
        
        ret, bw_img = cv2.threshold(hsv_thresh,0,255,cv2.THRESH_BINARY)
              
#        for x in range(0, 480):
#            for y in range(0, 640):
#                if bw_img[x, y] > 0:
#                    bw_img[x, y] = 1
#                              
        bw_img = numpy.divide(bw_img, 255)
        a = numpy.mean(bw_img[:, 0:320])
        b = numpy.mean(bw_img[:, 320:640])
        
        print bw_img
        print a
        print b

#        bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(),
#                                                  cv2.RETR_TREE,
#                                                  cv2.CHAIN_APPROX_SIMPLE)
#
#        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
#                                                  cv2.RETR_TREE,
#                                                  cv2.CHAIN_APPROX_SIMPLE)
##====================================================================================
#        for c in hsv_contours:
#            a = cv2.contourArea(c)
#            if a > 100.0:
#                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
                

        print '===='
        cv2.imshow("Image window", hsv_thresh)
#        cv2.imshow("Image window", bgr_thresh)

image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
