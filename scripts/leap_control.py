#!/usr/bin/env python2
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple Leap control application that controls drone movement

import rospy, sys, termios, tty, os
sys.path.insert(0, "/home/keith/Documents/LeapSDK/lib")
sys.path.insert(1, "/home/keith/Documents/LeapSDK/lib/x64")
import Leap
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

#wrapper for Twist message
class my_twist:
    def __init__():
        self.val = Twist()

    #reset the values of the Twist
    def reset(self):
        this.val.linear.x = 0.0
        this.val.linear.y = 0.0
        this.val.linear.z = 0.0
        this.val.angular.x = 0.0
        this.val.angular.y = 0.0
        this.val.angular.z = 0.0

#drone info class
class drone_info:
    def __init__(self):
        #initialize values
        #initialize publishers
        self.pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=10)
        self.pub_takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
        #get Leap Controller
        self.controller = Leap.Controller()
        #is the drone landed?
        self.is_landed = True
        #hand id
        self.old_hand_id = -1
        self.hand_id = -1
        #movement offsets (ROS axes)
        self.y_offset = 40
        self.z_offset = 60
        self.pitch_offset_forward = -0.1
        self.pitch_offset_back = 0.5
        self.l_roll_offset_left = 0.4
        self.l_roll_offset_right = -0.45
        self.r_roll_offset_left = 0.4
        self.r_roll_offset_right = -0.485
        self.z_rotation_offset_small = 0.15
        self.z_rotation_offset_large = 0.4
        self.x_magnitude = 0.2
        self.y_magnitude = 0.2
        self.z_magnitude = 0.4
        self.y_rotation = 0.4
        #y(LEAP) baseline
        self.y_base = -1
        #fist gesture detected
        self.fist = False

    def callback(self, msg):
        #process navdata
        #flight state
        state = msg.state
        if (state == 1 or state ==2):
            self.is_landed = True
        else:
            self.is_landed = False
        #battery percent
        if (msg.batteryPercent > 15):
            #call talker passing in drone info
            talker(self)
        else:
            rospy.logwarn("Low battery! Landing drone...")
            #land drone
            self.pub_land.publish()

def talker(di):

    #get frame from controller
    frame = controller.frame()
    #get the first hand detected
    hands = frame.hands
    #output string
    status = ""
    #is there movement?
    moved = False
    #twist variable
    vel_msg = my_twist()
    #reset twist parameters
    vel_msg.reset()
    if (len(hands) > 0):
        #get first hand
        first_hand = hands[0]
        if (di.hand_id != di.old_hand_id):
            #save previous hand id
            di.old_hand_id = di.hand_id
            di.hand_id = first_hand.id
            #get the LEAP y baseline
            di.y_base = first_hand.palm_position.y
        else:
            di.hand_id = first_hand.id
            
        pitch = first_hand.direction.pitch
        yaw = first_hand.direction.yaw
        roll = first_hand.palm_normal.roll
        #print("Palm position: {0},{1},{2}\nHand orientation: {3},{4},{5}".format(first_hand.palm_position.x,first_hand.palm_position.y,first_hand.palm_position.z,pitch,yaw,roll))

        #horizontal movement
        #ROS x axis (LEAP -z)
        if (pitch < di.pitch_offset_forward):
            #forward (ROS +x)
            status += "forward | "
            vel_msg.val.linear.x = di.x_magnitude
            moved = True
        elif (pitch > di.pitch_offset_back):
            #backward (ROS -x)
            status += "backward | "
            vel_msg.val.linear.x = -di.x_magnitude
            moved = True
        #ROS y axis (LEAP -x)
        if (first_hand.is_right):
            if (roll > di.r_roll_offset_left):
                #move left ROS y
                status += "left | "
                vel_msg.val.linear.y = di.y_magnitude
                moved = True
            elif (roll < di.r_roll_offset_right):
                #move right ROS -y
                status += "right | "
                vel_msg.val.linear.y = -di.y_magnitude
                moved = True
        else:
            if (roll > di.l_roll_offset_left):
                #move left ROS y
                status += "left | "
                vel_msg.val.linear.y = di.y_magnitude
                moved = True
            elif (roll < di.l_roll_offset_right):
                #move right ROS -y
                status += "right | "
                vel_msg.val.linear.y = -di.y_magnitude
                moved = True
        #vertical movement ROS z (LEAP y)
        if (first_hand.palm_position.y > di.y_base + di.z_offset):
            #move up
            status += "up | "
            vel_msg.val.linear.z = di.z_magnitude
            moved = True
        elif (first_hand.palm_position.y < di.y_base - di.z_offset):
            #move down
            status += "down | "
            vel_msg.val.linear.z = -di.z_magnitude
            moved = True
        #rotation ROS angular z (LEAP y rotation)
        if (first_hand.is_right):
            if (yaw > di.z_rotation_offset_small):
                #right
                status += "yaw right | "
                vel_msg.val.angular.z = -di.y_rotation
                moved = True
            elif (yaw < -di.z_rotation_offset_large):
                #left
                status += "yaw left | "
                vel_msg.val.angular.z = di.y_rotation
                moved = True
        else:
            if (yaw > di.z_rotation_offset_large):
                #right
                status += "yaw right | "
                vel_msg.val.angular.z = -di.y_rotation
                moved = True
            elif (yaw < -di.z_rotation_offset_small):
                #left
                status += "yaw left | "
                vel_msg.val.angular.z = di.y_rotation
                moved = True

        #takeoff and landing
        if (len(first_hand.fingers.extended()) < 2 and not di.fist):
            if (is_landed):
                di.pub_takeoff.publish()
                status += "Takeoff | "
                di.is_landed = not di.is_landed
            else:
                di.pub_land.publish()
                status += "Land | "
                di.is_landed = not di.is_landed
            di.fist = True
        elif (len(first_hand.fingers.extended()) < 2 and di.fist):
            #nothing
            nada = False
        else:
            di.fist = False
        
        #flip
        #if (gesture):
        #    try:
        #        flip = rospy.ServiceProxy('ardrone/setflightanimation', FlightAnim)
        #        resp = flip(19,0)
        #    except rospy.ServiceException, e:
        #        print "Service call failed: %s"%e
    #publish new twist
    di.pub_move.publish(vel_msg.val)
    #output activity
    rospy.loginfo(status)

def main():
    print("ARDrone 2.0 Leap Controller")
    #setup drone class
    my_drone = drone_info()
    #init node
    rospy.init_node("leap_control")
    #setup subscriber
    rospy.Subscriber("ardrone/navdata", Navdata, my_drone.callback)
    #spin
    rospy.spin()

if __name__ == '__main__':
    main()
