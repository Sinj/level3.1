#!/usr/bin/env python
import os
import time
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
import move_base_msgs.msg 

i = 0#global var, holds the state of the robot
def callback(data):
        global i
        if data.status.status >2: #if the robot failed/succeded in reaching goal store the result
            i = data.status.status
            
#def coords(x,y,z,w):#function to publish the movement coords to robot
#    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)#where it is publishing to
#    message = PoseStamped()
#    message.header.frame_id = '/map'
#    message.pose.position.x = x
#    message.pose.position.y = y
#    message.pose.orientation.z = z
#    message.pose.orientation.w = w
#    pub.publish(message)
    
#def makeclient():
#    rospy.loginfo("Creating base movement client.")
#    baseClient = actionlib.SimpleActionClient('/move_base_simple/goal',
#                                              PoseStamped)
 #   baseClient.wait_for_server() 
    
def makegoal(x,y,z,w):   
    print "in make goal"
    target = move_base_msgs.msg.MoveBaseGoal()
    target.target_pose.header.frame_id = "/map"
    print"header: target.target_pose.header.frame_id = {}".format(target.target_pose.header.frame_id)
    target.target_pose.pose.position.x = x
    print"target.target_pose.pose.position.x = {}, x = {} next ".format(target.target_pose.pose.position.x, x)
    target.target_pose.pose.position.y = y
    print "target.target_pose.pose.position.y = {}, y ={} next 2".format(target.target_pose.pose.position.y, y)
    target.target_pose.pose.orientation.z = z
    print"target.target_pose.pose.orientation.z = {}, z ={} next3".format(target.target_pose.pose.orientation.z, z)
    target.target_pose.pose.orientation.w = w
    print"target.target_pose.pose.orientation.w = {},w ={} next4".format(target.target_pose.pose.orientation.w, w)
    
   # print(target)
    print"leaving makegoal"
    return target
       
def robot_base():
   # end = False
    rospy.init_node('robot_base', anonymous=True)
    r = rospy.Rate(0.3) # .3hz
    global i
    k=0# The index for cords
    
    #make client
    rospy.loginfo("Creating base movement client.")
    baseClient = actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    baseClient.wait_for_server()#
    rospy.loginfo("stopped waiting.")
    
       
# Below is the open and read file   "/home/sinj/robot/src/base_move/script/cords.txt"
    cord=[]
    for line in open(os.path.dirname(os.path.realpath(__file__)) +'/cords.txt'): # takes the cord.txt from same directory
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
       
   
    target = makegoal(cord[k],cord[k+1],cord[k+2],cord[k+3])
    print "sending goal"
    baseClient.send_goal(target)

    print "goal sent"
    
    t = baseClient.get_state()
    print t
    
    print'goal coords: X:{} Y:{} Z:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2],cord[3], (len(cord)/4))

#    while not rospy.is_shutdown():
#        
#        s=5 # seconds, used for wait
#        if end == False:        
#            if i <3:
#                coords(cord[k],cord[k+1],cord[k+2],cord[k+3])
#                rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
#                print("keep moving to goal")  
#            if i == 3:
#                print'goal reached, waiting for {} seconds. at waypoint {}/{}'.format(s,((k+4)/4),(len(cord)/4))
#                i = 2
#                time.sleep(s)# waits for 4 seconds
#                print'k value before incroment {} and Lencord value {}'.format(k,len(cord))
#                if len(cord)-1 > k+4: 
#                    k = k +4 #move index to next set of cords
#                    print'next goal coords: X:{} Y:{} Z{}: W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
#                    print'moving to waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
#                else:
#                    print'the guiding has finished'
#                    end = True
#                    
#                    
#        if i == 4:
#            print'failed, retry- not implemented'
#            
#        if end:
#            if i <3:
#                coords(1.0,1.0,1.0,0.1)
#                rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
#                print'moving robot back to start'
#            if i == 3:
#                 print'program end'
#                 rospy.signal_shutdown("program end")
#                 
#        r.sleep()
    
if __name__ == '__main__':
    try:
        robot_base()
    except rospy.ROSInterruptException: pass
 
