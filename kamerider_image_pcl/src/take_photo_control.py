#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import sys, select, termios, tty
from std_msgs.msg import String
from turtlebot_msgs.srv import SetFollowState

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node("take_photo_control", anonymous=True)
    rospy.loginfo("Waiting for service...")
    pub = rospy.Publisher("/take_photo", String, queue_size=1)
  
    while(1):
        key = getKey()
        try:    
            if key == 's':
                pub.publish("start")
            if key == 'q':
                msg = False
                response = set_state(msg)
        except:
            print "ERROR"
if __name__ == '__main__':
    main()

