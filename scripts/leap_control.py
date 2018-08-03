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
#sys.path.insert(0, "../leap_x64")
sys.path.insert(0, "/home/keith/Documents/LeapSDK/lib")
sys.path.insert(1, "/home/keith/Documents/LeapSDK/lib/x64")
import Leap
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def talker():
    print("ARDrone 2.0 Leap Controller")
    print("Controls:")
    pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=10)
    pub_takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
    #start node
    rospy.init_node('leap_control', anonymous=True)
    #get Leap Controller
    controller = Leap.Controller()
    #is the drone landed?
    is_landed = True
    #hand id
    old_hand_id = -1
    hand_id = -1
    #movement offsets (ROS axes)
    y_offset = 40
    z_offset = 60
    pitch_offset_forward = -0.1
    pitch_offset_back = 0.5
    l_roll_offset_left = 0.4
    l_roll_offset_right = -0.45
    r_roll_offset_left = 0.4
    r_roll_offset_right = -0.485
    z_rotation_offset_small = 0.15
    z_rotation_offset_large = 0.4
    x_magnitude = 0.2
    y_magnitude = 0.2
    z_magnitude = 0.4
    y_rotation = 0.4
    #y(LEAP) baseline
    y_base = -1
    #fist gesture detected
    fist = False
    #set loop rate
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        #get frame from controller
        frame = controller.frame()
        #get the first hand detected
        hands = frame.hands
        #output string
        status = ""
        #is there movement?
        moved = False
        #twist variable
        vel_msg = Twist()
        #reset twist parameters
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        if (len(hands) > 0):
            #get first hand
            first_hand = hands[0]
            if (hand_id != old_hand_id):
                #save previous hand id
                old_hand_id = hand_id
                hand_id = first_hand.id
                #get the LEAP y baseline
                y_base = first_hand.palm_position.y
            else:
                hand_id = first_hand.id
                
            pitch = first_hand.direction.pitch
            yaw = first_hand.direction.yaw
            roll = first_hand.palm_normal.roll
            #print("Palm position: {0},{1},{2}\nHand orientation: {3},{4},{5}".format(first_hand.palm_position.x,first_hand.palm_position.y,first_hand.palm_position.z,pitch,yaw,roll))

            #horizontal movement
            #ROS x axis (LEAP -z)
            if (pitch < pitch_offset_forward):
                #forward (ROS +x)
                status += "forward | "
                vel_msg.linear.x = x_magnitude
                moved = True
            elif (pitch > pitch_offset_back):
                #backward (ROS -x)
                status += "backward | "
                vel_msg.linear.x = -x_magnitude
                moved = True
            #ROS y axis (LEAP -x)
            if (first_hand.is_right):
                if (roll > r_roll_offset_left):
                    #move left ROS y
                    status += "left | "
                    vel_msg.linear.y = y_magnitude
                    moved = True
                elif (roll < r_roll_offset_right):
                    #move right ROS -y
                    status += "right | "
                    vel_msg.linear.y = -y_magnitude
                    moved = True
            else:
                if (roll > l_roll_offset_left):
                    #move left ROS y
                    status += "left | "
                    vel_msg.linear.y = y_magnitude
                    moved = True
                elif (roll < l_roll_offset_right):
                    #move right ROS -y
                    status += "right | "
                    vel_msg.linear.y = -y_magnitude
                    moved = True
            #vertical movement ROS z (LEAP y)
            if (first_hand.palm_position.y > y_base + z_offset):
                #move up
                status += "up | "
                vel_msg.linear.z = z_magnitude
                moved = True
            elif (first_hand.palm_position.y < y_base - z_offset):
                #move down
                status += "down | "
                vel_msg.linear.z = -z_magnitude
                moved = True
            #rotation ROS angular z (LEAP y rotation)
            if (first_hand.is_right):
                if (yaw > z_rotation_offset_small):
                    #right
                    status += "yaw right | "
                    vel_msg.angular.z = -y_rotation
                    moved = True
                elif (yaw < -z_rotation_offset_large):
                    #left
                    status += "yaw left | "
                    vel_msg.angular.z = y_rotation
                    moved = True
            else:
                if (yaw > z_rotation_offset_large):
                    #right
                    status += "yaw right | "
                    vel_msg.angular.z = -y_rotation
                    moved = True
                elif (yaw < -z_rotation_offset_small):
                    #left
                    status += "yaw left | "
                    vel_msg.angular.z = y_rotation
                    moved = True

            #takeoff and landing
            if (len(first_hand.fingers.extended()) < 2 and not fist):
                if (is_landed):
                    pub_takeoff.publish()
                    status += "Takeoff | "
                    is_landed = not is_landed
                else:
                    pub_land.publish()
                    status += "Land | "
                    is_landed = not is_landed
                fist = True
            elif (len(first_hand.fingers.extended()) < 2 and fist):
                #nothing
                nada = False
            else:
                fist = False
            
            #flip
            #if (gesture):
            #    try:
            #        flip = rospy.ServiceProxy('ardrone/setflightanimation', FlightAnim)
            #        resp = flip(19,0)
            #    except rospy.ServiceException, e:
            #        print "Service call failed: %s"%e
        #hover
        if (not moved):
            #auto hover
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
        #publish new twist
        pub_move.publish(vel_msg)
        #output activity
        rospy.loginfo(status)
        #wait for loop time
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
