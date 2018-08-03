#!/usr/bin/env python
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

## Simple key control application that controls drone movement
# source for key press detection
# www.jonwitts.co.uk/archives/896

import rospy, sys, termios, tty, os, time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def talker():
    #instructions
    print("ARDrone 2.0 Key Controller")
    print("Controls:")
    print("  q,a: fly up & down")
    print("  i,j,k,l: fly horizontally")
    print("  u,o: rotate yaw")
    print("  s: takeoff")
    print("  d: land")
    print("  h: hover")
    print("  c: quit")
    #ROS publishers
    pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=10)
    pub_takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
    pub_reset = rospy.Publisher('ardrone/reset', Empty, queue_size=10)
    #create node
    rospy.init_node('key_control', anonymous=True)
    #update rate
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        #get input from keyboard
        char = getch()
        #output string
        status = ""
        #twist variable
        vel_msg = Twist()
        #reset twist parameters
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        #quit
        if (char == "c"):
            exit(0)
        #takeoff and landing keys
        if (char == "s"):
            status += "Takeoff | "
            pub_takeoff.publish()
        elif (char == "d"):
            status += "Land | "
            pub_land.publish()
        #horizontal movement
        #x axis
        if (char == "i"):
            status += "Forward | "
            vel_msg.linear.x = 0.2
        elif (char == "k"):
            status += "Back | "
            vel_msg.linear.x = -0.2
        #y axis
        if (char == "l"):
            status += "Right | "
            vel_msg.linear.y = -0.2
        elif (char == "j"):
            status += "Left | "
            vel_msg.linear.y = 0.2
        #vertical movement
        if (char == "q"):
            status += "Up | "
            vel_msg.linear.z = 0.4
        elif (char == "a"):
            status += "Down | "
            vel_msg.linear.z = -0.4
        #rotation
        if (char == "u"):
            status += "Turn Left"
            vel_msg.angular.z = 0.2
        elif (char == "o"):
            status += "Turn Right"
            vel_msg.angular.z = -0.2
        else:
            vel_msg.angular.z = 0.0
        #hover
        if (char == "h" or status == ""):
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
