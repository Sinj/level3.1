#!/usr/bin/env python
import os
import flir_pantilt_d46.msg
import time
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped 
import actionlib
import move_base_msgs.msg 

sqrt = 0.0
    
def makegoal(x,y,z,w):   
    
    target = move_base_msgs.msg.MoveBaseGoal()
    target.target_pose.header.frame_id = "/map"
    target.target_pose.pose.position.x = x
    target.target_pose.pose.position.y = y
    target.target_pose.pose.orientation.z = z
    target.target_pose.pose.orientation.w = w 
    return target
    
    
    
def wherehuman(data):
 #get the distance the humans is away from  the robot and store it  
    global sqrt
    toBeSqrt = (data.pose.position.x * data.pose.position.x) + (data.pose.position.y * data.pose.position.y)   
    sqrt = math.sqrt(toBeSqrt) 
  
def robot_base():
   
    rospy.init_node('robot_base', anonymous=True)
    r = rospy.Rate(0.3) # .3hz
    file1 = '/cords4.txt'
    global sqrt
    k=0# The index for cords
    end = False   #signal the end for the program
    humanDistance = 4  # stores max distance the human can be from the robot
    waitTime = 20.0    #in seconds
    
        #make client
    print"making move client"
    baseClient = actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    baseClient.wait_for_server()#
    ptuclient = actionlib.SimpleActionClient('SetPTUState',flir_pantilt_d46.msg.PtuGotoAction)    
    ptuclient.wait_for_server()
    
    ptuforward = flir_pantilt_d46.msg.PtuGotoGoal()
    ptuforward.pan = 0
    ptuforward.pan_vel = 60

    ptubwrd = flir_pantilt_d46.msg.PtuGotoGoal()
    ptubwrd.pan = -180
    ptubwrd.pan_vel = 60    
    
# Below is the open and read file
    print "reading cords from {}".format(file1)
    cord=[]
    for line in open(os.path.dirname(os.path.realpath(__file__)) + file1):
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
#---------------------------------------------------------    
    
    
    
    i = 0
    print'goal coords: X:{} Y:{} Z:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2],cord[3], (len(cord)/4))
    
    while not rospy.is_shutdown():                            
        if end == False:        
            if i <3:
                ptuclient.send_goal_and_wait(ptuforward)

                print("moving to goal") 
                baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3]))    #send goal cords
                i = baseClient.get_state()#get the current state of robot
                 
            if i == 3:
                print'goal reached, waiting for human. \nCurrently at waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                sqrt = 0.0
                waitForHuman = True
                timer = time.time()
                i=0
                ptuclient.send_goal_and_wait(ptubwrd)
                while waitForHuman:                   
                                       
                    print "waiting for {:.2f} seconds ".format(((timer+waitTime) - time.time()))
                    rospy.Subscriber("/human/transformed", PoseStamped, wherehuman)
                    r.sleep()
                                        
                    if sqrt >0.5 and sqrt <= humanDistance:
                        print 'human spotted at {}'.format(sqrt)
                        if len(cord)-1 > k+4: 
                           sqrt = 0.0
                           k = k +4 #move index to next set of cords
                           print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                           print'moving to waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))                          
                        else:
                            print'the guiding has finished'
                            end = True                        
                        waitForHuman = False
                        
                    elif (timer+waitTime) < time.time():
                         print "human has not appered after {} seconds".format(waitTime)
                         r.sleep()
                         if  k-4 >= 0:
                             print"Robot will go back to prior waypoint and wait"
                             k = k -4 #move index to prior set of cords
                             print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                             print'moving back to prior waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                         else:
                             print 'no human is following robot, will go back to start' 
                             end = True
                         waitForHuman = False
                    else: 
                        print 'waiting for human'               
                                      
        if i == 4:
            print'failed, retry- not implemented'
        if end:
            if i <3:
                print'moving robot back to start'
                baseClient.send_goal_and_wait(makegoal(-2.0,0.0,1.0,0.1))
                i = baseClient.get_state() #get the current state of robot
                
            if i >=3:
                 print'program end'
                 rospy.signal_shutdown("program end")                 
        r.sleep()
    
if __name__ == '__main__':
    try:
        robot_base()
    except rospy.ROSInterruptException: pass
