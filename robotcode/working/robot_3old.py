#!/usr/bin/env python
import os
import flir_pantilt_d46.msg
import mary_tts.msg
import time
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped 
import actionlib
import move_base_msgs.msg 
import random

sqrt = 0.0 # hold the sqrt distance of the human from the robot
    
def makegoal(x,y,z,w): #makes move goal & return it 
    target = move_base_msgs.msg.MoveBaseGoal()
    target.target_pose.header.frame_id = "/map"
    target.target_pose.pose.position.x = x
    target.target_pose.pose.position.y = y
    target.target_pose.pose.orientation.z = z
    target.target_pose.pose.orientation.w = w 
    return target  

def wherehuman(data):
    #get the euclidean distance the humans is away from  the robot and store it  
    print data
    global sqrt
    toBeSqrt = (data.pose.position.x * data.pose.position.x) + (data.pose.position.y * data.pose.position.y)   
    sqrt = math.sqrt(toBeSqrt) 
  
def robot_base():
    rospy.init_node('robot_base', anonymous=True)
    r = rospy.Rate(0.3) # .3hz
    cordsfile = '/cords4.txt'
    speakfile = '/tospeak.txt'
    global sqrt
    robotstate = 0          # holds the state of the robot
    k=0                     # The index for cords
    end = False             # signal for the end for the program
    humanMaxDistance = 3.5  # stores max distance the human can be from the robot
    humanMinDistance = 0.5  # stores min distance the human can be from the robot
    robotWaitTime = 20.0    # in seconds
    lookforhu = True        # holds true if it is looking for human while moving
    checkhumtime = 20.0    # time in second how long wait before check for human
    
        #make client
    print"making clients"
    baseClient = actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    baseClient.wait_for_server()#
        #make PTU client
    ptuclient = actionlib.SimpleActionClient('SetPTUState',flir_pantilt_d46.msg.PtuGotoAction)    
    ptuclient.wait_for_server()
        #Make speaking client
    maryclient = actionlib.SimpleActionClient('speak',mary_tts.msg.maryttsAction)    
    maryclient.wait_for_server()
        #face foward PTU goal 
    ptuforward = flir_pantilt_d46.msg.PtuGotoGoal()
    ptuforward.pan = 0
    ptuforward.pan_vel = 60
        #face backward PTU goal
    ptubwrd = flir_pantilt_d46.msg.PtuGotoGoal()
    ptubwrd.pan = -180
    ptubwrd.pan_vel = 60
        #speaking goal    
    speak = mary_tts.msg.maryttsGoal()    

        # Below is the open and read file- first as float, second as string
    print "reading cords from {}".format(cordsfile)
    cord=[]
    for line in open(os.path.dirname(os.path.realpath(__file__)) + cordsfile):
       cord.append(np.array([float(val) for val in line.rstrip('\n').split(' ') if val != '']))
       
    print "reading cords from {}".format(speakfile)           
    with open(os.path.dirname(os.path.realpath(__file__)) + speakfile, "r") as f:
        speakarray = map(str.rstrip, f)       
           
    speak.text = speakarray[0]  # great people              
    maryclient.send_goal_and_wait(speak)
    print'Going to start at coords: X:{} Y:{} Z:{} W:{} and there are {} waypoints'.format(cord[0],cord[1],cord[2],cord[3], (len(cord)/4)-1)
    speak.text = speakarray[1]  #inform them of action              
    maryclient.send_goal(speak)
    baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3]))
    speak.text = speakarray[2]   #ask them to move behind robot             
    maryclient.send_goal_and_wait(speak)                                                                                                                
    k = k +4 #move index to next set of cords
    
    while not rospy.is_shutdown():                            
        if end == False:        
            if robotstate <3:
                speak.text = speakarray[random.randint(3,7)]#selet random moving line                
                maryclient.send_goal(speak)#speak selected
                ptuclient.send_goal_and_wait(ptuforward)#face topcam forward
                print("moving to goal")
                
                if lookforhu: # check if moving to next goal
                    baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                       cord[k+2],cord[k+3],rospy.Duration.from_sec(checkhumtime)))#send goal cords
                else:
                    baseClient.send_goal_and_wait(makegoal(cord[k],cord[k+1],
                                                           cord[k+2],cord[k+3]))
                robotstate = baseClient.get_state()#get the current state of robot
                
                if robotstate == 2 and lookforhu:
                    print "checking if human is still following"
                    ptuclient.send_goal_and_wait(ptubwrd) # rotate the topcam to see if human is there
######################################################                    
                    sqrt = 0.0
                    waitForHuman = True
                    timer = time.time()
                    tim = 5
                    while waitForHuman:
                        print "waiting for {:.2f} seconds ".format(((timer+tim) - time.time()))
                        rospy.Subscriber("/people_tracker/pose", PoseStamped, wherehuman)
                        
                    if sqrt >humanMinDistance and sqrt <= humanMaxDistance:
                        print 'human spotted at {}, resume moving to goal'.format(sqrt)
                        sqrt = 0.0
                        waitForHuman = False
                    elif (timer+tim) < time.time(): # ether back to prior waypoint or if not prior waypoint end guide
                        print" human not spotted"
                        lookforhu = False
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
                    ptuclient.send_goal_and_wait(ptuforward)
                    
                # add duration to wait, if it break out of wait then turn the top camera, then do human checking, no human wait for X, still no human , go bk to prior wait point 

                 
            if robotstate == 3:
                print'goal reached, waiting for human. \nCurrently at waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                speak.text = speakarray[random.randint(8,10)] #select randomly from we at waypoint               
                maryclient.send_goal(speak)#speak selection 
                
                sqrt = 0.0
                waitForHuman = True
                timer = time.time()
                robotstate=0
                
                speak.text = speakarray[random.randint(11,13)]                
                maryclient.send_goal(speak)
                ptuclient.send_goal_and_wait(ptubwrd)#look behind with topcam                
                while waitForHuman:                  
                    print "waiting for {:.2f} seconds ".format(((timer+robotWaitTime) - time.time()))
                    rospy.Subscriber("/people_tracker/pose", PoseStamped, wherehuman)
                    r.sleep()
                                        
                    if sqrt >humanMinDistance and sqrt <= humanMaxDistance:
                        print 'human spotted at {}'.format(sqrt)
                        if len(cord)-1 > k+4: 
                           sqrt = 0.0
                           speak.text = speakarray[random.randint(18,21)]                
                           maryclient.send_goal(speak)
                           k = k +4 #move index to next set of cords
                           print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                           print'moving to waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                           lookforhu = True
                        else:                            
                            print'the guiding has finished'
                            end = True 
                            lookforhu = False
                        waitForHuman = False
                        
                    elif (timer+robotWaitTime) < time.time():
                         print "human has not appered after {} seconds".format(robotWaitTime)
                         r.sleep()
                         speak.text = speakarray[random.randint(22,24)]                
                         maryclient.send_goal_and_wait(speak)
                         if  k-4 >= 0:
                             print"Robot will go back to prior waypoint and wait"
                             k = k -4 #move index to prior set of cords
                             print'next goal coords: X:{} Y:{} Z:{} W:{}'.format(cord[k],cord[k+1],cord[k+2],cord[k+3])
                             print'moving back to prior waypoint {}/{}'.format(((k+4)/4),(len(cord)/4))
                             speak.text = speakarray[random.randint(25,26)]                
                             maryclient.send_goal(speak)
                         else:
                             speak.text = speakarray[27]                
                             maryclient.send_goal(speak)                             
                             print 'no human is following robot, will go back to start' 
                             end = True
                         waitForHuman = False
                         lookforhu = False
                    else: 
                        print 'waiting for human'               
                                      
        if robotstate == 4:
            print'failed, retry- not implemented'
        if end:
            ptuclient.send_goal_and_wait(ptuforward)
            if robotstate <3:
                print'moving robot back to start'
                speak.text = speakarray[random.randint(14,15)]                
                maryclient.send_goal_and_wait(speak)
                speak.text = speakarray[16]                
                maryclient.send_goal_and_wait(speak)
                baseClient.send_goal_and_wait(makegoal(-2.0,0.0,1.0,0.1))
                robotstate = baseClient.get_state() #get the current state of robot
                
            if robotstate >=3:
                 speak.text = speakarray[17]
                 maryclient.send_goal_and_wait(speak)
                 print'program end'
                 rospy.signal_shutdown("program end")                 
        r.sleep()
    
if __name__ == '__main__':
    try:
        robot_base()
    except rospy.ROSInterruptException: pass
