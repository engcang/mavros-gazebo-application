#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Joy
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import Thrust

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle

global joy_check
joy_check=0
global mav_check
mav_check=0

global d2r
global r2d
global max_rate_x
global max_rate_y
global max_vel_x
global max_vel_y
global max_vel_z
global yaw_rate
yaw_rate = 2

r2d = 180/np.pi
d2r = np.pi/180
max_rate_x = 360 * d2r
max_rate_y = 360 * d2r
max_vel_x = 10
max_vel_y = 10
max_vel_z = 4

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.parent_frame_id = rospy.get_param("/mobile_robot_vel_topic", "/mobile_base/commands/velocity")
        self.mobile_robot_vel_pub = rospy.Publisher(self.parent_frame_id, Twist, queue_size=10)
        self.position_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.rate_pub = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(30)
        self.mode = 2 #default is mode 2
        self.mission = 0
        self.hold = 0 #to stop sending input shortly
        self.mobile_robot = 0

    def pose_callback(self, msg):
        global mav_check
        self.truth=msg.pose.position
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        mav_check=1

    def joy_callback(self, msg):
        self.joy = msg
        global joy_check
        if len(self.joy.axes)>0 or len(self.joy.buttons)>0 :
            joy_check=1
            if self.joy.buttons[1]==1:
                self.mission = self.mission+1
            if self.joy.buttons[4]==1:
                self.mode=1
            if self.joy.buttons[5]==1:
                self.mode=2
            if self.joy.buttons[2]==1:
                self.arming(True)
            if self.joy.buttons[3]==1:
                self.offboarding(base_mode=0, custom_mode="OFFBOARD")
            if self.joy.buttons[0]==1:
                self.hold = self.hold+1
            if self.joy.buttons[6]==1:
                self.mobile_robot = 1
            if self.joy.buttons[7]==1:
                self.mobile_robot = 0

def input(rbt):
    global d2r
    global r2d
    global max_rate_x
    global max_rate_y
    global max_vel_x
    global max_vel_y
    global max_vel_z
    global yaw_rate

    if rbt.mobile_robot==0:
        if rbt.hold%2==0:
            if rbt.mission%2 == 0: #position mode
                pose_input=PoseStamped()

                ##Mode 2, default
                #joy_axes: {pitch: 4, roll: 3, yaw: 0, vertical: 1}
                if rbt.mode==2:
                    pose_input.pose.position.x= rbt.truth.x + ( rbt.joy.axes[4]*max_vel_x)*cos(rbt.yaw) - ( rbt.joy.axes[3]*max_vel_y)*sin(rbt.yaw)
                    pose_input.pose.position.y= rbt.truth.y + ( rbt.joy.axes[3]*max_vel_y)*cos(rbt.yaw) + ( rbt.joy.axes[4]*max_vel_x)*sin(rbt.yaw)
                    pose_input.pose.position.z= rbt.truth.z + ( rbt.joy.axes[1]*max_vel_z)
                    yaw_input = rpy_saturation(rbt.yaw + yaw_rate*(rbt.joy.axes[0]))
                    qq = quaternion_from_euler(0,0,yaw_input)
                    pose_input.pose.orientation.x = qq[0]
                    pose_input.pose.orientation.y = qq[1]
                    pose_input.pose.orientation.z = qq[2]
                    pose_input.pose.orientation.w = qq[3]
                ##Mode 1
                #joy_axes: {pitch: 1, roll: 3, yaw: 0, vertical: 4}
                elif rbt.mode==1:
                    pose_input.pose.position.x= rbt.truth.x + ( rbt.joy.axes[1]*max_vel_x)*cos(rbt.yaw) - ( rbt.joy.axes[3]*max_vel_y)*sin(rbt.yaw)
                    pose_input.pose.position.y= rbt.truth.y + ( rbt.joy.axes[3]*max_vel_y)*cos(rbt.yaw) + ( rbt.joy.axes[1]*max_vel_x)*sin(rbt.yaw)
                    pose_input.pose.position.z= rbt.truth.z + ( rbt.joy.axes[4]*max_vel_z)
                    yaw_input = rpy_saturation(rbt.yaw + yaw_rate*(rbt.joy.axes[0]))
                    qq = quaternion_from_euler(0,0,yaw_input)
                    pose_input.pose.orientation.x = qq[0]
                    pose_input.pose.orientation.y = qq[1]
                    pose_input.pose.orientation.z = qq[2]
                    pose_input.pose.orientation.w = qq[3]

                print("Mode < %d > now, press L1 or R1 to change, Press Button[2],[3] to arm/disarm"%rbt.mode)
                print("Mission:< %d >, Input : X: %.2f  Y: %.2f  Z: %.2f  Yaw: %.2f "%(rbt.mission%2, pose_input.pose.position.x, pose_input.pose.position.y, pose_input.pose.position.z, yaw_input))
                print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(rbt.truth.x, rbt.truth.y, rbt.truth.z))
                print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi

                pose_input.header.stamp = rospy.Time.now()
                rbt.position_pub.publish(pose_input)

            elif rbt.mission%2 == 1: #rate mode

                rate_input=TwistStamped()
                thrust_input=Thrust()

        #        ##Mode 2, default
        #        #joy_axes: {pitch: 4, roll: 3, yaw: 0, vertical: 1}
                if rbt.mode==2:
                    rate_input.twist.angular.y = -rbt.joy.axes[3]*max_rate_x #x,y are conversed... I hate px4
                    rate_input.twist.angular.x = -rbt.joy.axes[4]*max_rate_y
                    rate_input.twist.angular.z = yaw_rate*(rbt.joy.axes[0])
                    thrust_input.thrust = 0.57 + (rbt.joy.axes[1])*0.4
        #        ##Mode 1
        #        #joy_axes: {pitch: 1, roll: 3, yaw: 0, vertical: 4}
                elif rbt.mode==1:
                    rate_input.twist.angular.y = -rbt.joy.axes[3]*max_rate_x #x,y are conversed... I hate px4
                    rate_input.twist.angular.x = -rbt.joy.axes[1]*max_rate_y
                    rate_input.twist.angular.z = yaw_rate*(rbt.joy.axes[0])
                    thrust_input.thrust = 0.57 + (rbt.joy.axes[4])*0.4

                print("Mode < %d > now, press L1 or R1 to change, Press Button[2],[3] to arm/disarm"%rbt.mode)
                print("Mission:< %d >, Input : R: %.2f  P: %.2f  Y: %.2f  thrust: %.2f "%(rbt.mission%2, rate_input.twist.angular.x*r2d, rate_input.twist.angular.y*r2d, rate_input.twist.angular.z*r2d, thrust_input.thrust))
                print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(rbt.truth.x, rbt.truth.y, rbt.truth.z))
                print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi

                thrust_input.header.stamp = rate_input.header.stamp = rospy.Time.now()
                rbt.rate_pub.publish(rate_input)
                rbt.thrust_pub.publish(thrust_input)
        else:
            print("Hold now, press Button[0] to control again")
            print("Position(Meter): X: %.2f Y: %.2f Z: %.2f "%(rbt.truth.x, rbt.truth.y, rbt.truth.z))
            print("Angle(Degree): roll: %.2f pitch: %.2f yaw: %.2f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi
    else:
        k_vel_input = Twist()
        k_vel_input.linear.x= rbt.joy.axes[4]
        k_vel_input.angular.z = yaw_rate*(rbt.joy.axes[0])

        rbt.mobile_robot_vel_pub.publish(k_vel_input)

##############################################################################################

mav_ctr = robot()
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if joy_check==1 and mav_check==1:
                input(mav_ctr)
                mav_ctr.rate.sleep()
            else: 
                mav_ctr.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
