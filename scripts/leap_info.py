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

## Simple Leap control application that prints hand info on the first hand

import rospy, sys, tty, os
sys.path.insert(0, "/home/keith/Documents/LeapSDK/lib")
sys.path.insert(1, "/home/keith/Documents/LeapSDK/lib/x64")
import Leap

def talker():
    print("Leap info")
    #start node
    rospy.init_node('leap_info', anonymous=True)
    #get Leap Controller
    controller = Leap.Controller()
    #set loop rate
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        #get frame from controller
        frame = controller.frame()
        #get the first hand detected
        hands = frame.hands
        if (len(hands) > 0):
            #get first hand
            first_hand = hands[0]
                
            pitch = first_hand.direction.pitch
            yaw = first_hand.direction.yaw
            roll = first_hand.palm_normal.roll
            print("Palm position: {0},{1},{2}\nHand orientation: {3},{4},{5}".format(first_hand.palm_position.x,first_hand.palm_position.y,first_hand.palm_position.z,pitch,yaw,roll))
        #wait for loop time
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
