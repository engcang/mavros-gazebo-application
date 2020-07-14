#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from mavros_msgs.srv import SetMode, CommandBool

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


global joy_check
joy_check=0

global d2r
global r2d
global max_vel_x
global max_vel_y
global max_vel_z
global yaw_rate
yaw_rate = 3

r2d = 180/np.pi
d2r = np.pi/180
max_ang_x = 15
max_ang_y = 15
max_ang_z = 15

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(30)

        self.joy = Joy()
        self.mode = 2 #default is mode 2

    def pose_callback(self, msg):
        self.truth=msg.pose.position
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def joy_callback(self, msg):
        self.joy = msg
        global joy_check
        if len(self.joy.axes)>0 or len(self.joy.buttons)>0 :
            joy_check=1
            if self.joy.buttons[4]==1:
                self.mode=1
            if self.joy.buttons[5]==1:
                self.mode=2
            if self.joy.buttons[2]==1:
                self.arming(True)
            if self.joy.buttons[3]==1:
                self.offboarding(base_mode=0, custom_mode="OFFBOARD")

def input(rbt):
    global d2r
    global r2d
    global max_ang_x
    global max_ang_y

    vel_input=TwistStamped()
##Mode 2, default
#joy_axes: {pitch: 4, roll: 3, yaw: 0, vertical: 1}
    if rbt.mode==2:
        vel_input.twist.linear.x= ( rbt.joy.axes[4]*max_ang_x)
        vel_input.twist.linear.y= ( rbt.joy.axes[3]*max_ang_y)
        vel_input.twist.linear.z= ( rbt.joy.axes[1]*max_ang_z)
        vel_input.twist.angular.z = yaw_rate*(rbt.joy.axes[0])
##Mode 1
#joy_axes: {pitch: 1, roll: 3, yaw: 0, vertical: 4}
    elif rbt.mode==1:
        vel_input.twist.linear.x= ( rbt.joy.axes[1]*max_ang_x)
        vel_input.twist.linear.y= ( rbt.joy.axes[3]*max_ang_y)
        vel_input.twist.linear.z= ( rbt.joy.axes[4]*max_ang_z)
        vel_input.twist.angular.z = yaw_rate*(rbt.joy.axes[0])

    print("Mode < %d > now, press L1 or R1 to change, Press Button[2] to arm/disarm"%rbt.mode)
    print("Input : X: %.2f  Y: %.2f  Z: %.2f  Yaw: %.2f "%(vel_input.twist.linear.x, vel_input.twist.linear.y, vel_input.twist.linear.z, vel_input.twist.angular.z))
    print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(rbt.truth.x, rbt.truth.y, rbt.truth.z))
    print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi

    vel_input.header.stamp = rospy.Time.now()
    rbt.local_vel_pub.publish(vel_input)

##############################################################################################

alpha = robot()
alpha.joy_callback(alpha.joy)
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if joy_check==1:
                input(alpha)
                alpha.rate.sleep()
            else: 
                alpha.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass