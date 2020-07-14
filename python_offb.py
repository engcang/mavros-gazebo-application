#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 14 06:02:03 2019

@author: mason
"""

''' import libraries '''
import time
from math import sqrt, pow

import rospy
import roslib
import mavros

from geometry_msgs.msg  import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

from mavros_msgs.srv import SetMode, CommandBool

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) # topic msg Publish
        self.local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state',State, self.state_callback) # topic msg subscribe
        self.position_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.current_state = State()
        #self.prev_state = State()
        self.pose = Odometry()
        self.vel = TwistStamped()
        self.vel_ = self.vel.twist.linear

        self.rate = rospy.Rate(25)

    def state_callback(self, msg):
        self.current_state = msg
    def position_callback(self, msg):
        self.pose = msg
        self.pose_= self.pose.pose.pose.position

    def set_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self.local_pos_pub.publish(goal)

    def vel_goal(self, goal):
        goal_ = goal.pose.position

        self.vel_.x = 0.5*(goal_.x - self.pose_.x)
        self.vel_.y = 0.5*(goal_.y - self.pose_.y)
        self.vel_.z = 0.5*(goal_.z - self.pose_.z)

        if self.vel_.x > 2:
            self.vel_.x = 2
        if self.vel_.x < -2:
            self.vel_.x = -2
        if self.vel_.y > 2:
            self.vel_.y = 2
        if self.vel_.y < -2:
            self.vel_.y = -2
        if self.vel_.z > 2:
            self.vel_.z = 2
        if self.vel_.z < -2:
            self.vel_.z = -2

        self.vel.header.stamp = rospy.Time.now()
        self.local_vel_pub.publish(self.vel)

def dist(goal, odom_pose):
    now = odom_pose.pose.pose.position
    goal_ = goal.pose.position
    return sqrt(pow(now.x-goal_.x,2) + pow(now.y-goal_.y,2) + pow(now.z-goal_.z,2))


def tic(): 
    global starttime
    starttime=time.time()

def toc():
    nowtime=time.time()
    #print("toc: %f"%(nowtime-starttime))
    return nowtime-starttime

##############################################################################################
#data = State()
px4 = robot()
#px4.state_callback(data)

goal = PoseStamped()
goal_ = goal.pose.position

goal_.x=0
goal_.y=0
goal_.z=4

time.sleep(1) #wait 1 second to assure that all data comes in

off_check=0
arm_check=0

set_goal=0

''' main '''
if __name__ == '__main__':
 while 1:
    try:
        # send a few setpoints before starting
        for i in range(0,100):
            px4.set_goal(goal)

        while not px4.current_state.connected: # wait for FCU connection
            px4.rate.sleep()
         
        tic()
        while not rospy.is_shutdown():
            if px4.current_state.mode != "OFFBOARD" and (toc() > 2) :
                px4.offboarding(base_mode=0, custom_mode="OFFBOARD")
                off_check=0
                tic()
            elif not px4.current_state.armed and (toc() > 2) :
                px4.arming(True)
                arm_check=0
                tic()
            
            if px4.current_state.mode == "OFFBOARD" and off_check==0:
                rospy.loginfo("offboard enabled : %r" %px4.current_state.mode)
                off_check=1
            elif px4.current_state.armed and arm_check==0:
                rospy.loginfo("Vehicle armed : %s" %px4.current_state.armed)
                arm_check=1

            if dist(goal, px4.pose) < 0.3:
                if set_goal == 0:
                    goal_.x=4
                    goal_.y=0
                    goal_.z=4
                    set_goal = 1
                elif set_goal == 1:
                    goal_.x=4
                    goal_.y=4
                    goal_.z=4
                    set_goal = 2
                elif set_goal == 2:
                    goal_.x=0
                    goal_.y=4
                    goal_.z=4
                    set_goal = 3
                elif set_goal == 3:
                    goal_.x=0
                    goal_.y=0
                    goal_.z=4
                    set_goal = 0

            #px4.set_goal(goal) #by position command
            px4.vel_goal(goal) #by velocity command
            px4.rate.sleep()

    except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
        sys.exit(0)
    #except:
    #    print("exception")
    #    pass
