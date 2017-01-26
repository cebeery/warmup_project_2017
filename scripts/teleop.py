#!/usr/bin/env python

"""This script allows for basic keyboard teleop of Neato"""

import rospy
import tty, select, sys, termios
from geometry_msgs.msg import Twist

s = 0.4 #speed
keyBindings = { 
               'w':(s,0),
               'a':(0,-s),
               's':(-s,0),
               'd':(0,s)
              }
   
def getKey():
    """return keyboard stroke when off terminal"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def makeCmdMessage(key):
    """translate key press to cmd_vel message"""
    cmd = Twist()

    if key in keyBindings:
        print key
        cmd.linear.x = keyBindings[key][0] #forward/back
        cmd.angular.z = keyBindings[key][1] #turn
    else
        #don't change Twist 0.0 defaults
        print "Key != wasd: pause motion

    return cmd


#keyboard prep stuff
settings = termios.tcgetattr(sys.stdin)
key = None

#iniitalize ros communication
rospy.init_node('teleop')
publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#loop key command updates
rate = rospy.Rate(10) #****check required rate
while key != '\x03':
    key = getKey()

    cmd = makeCmdMessage(key)
    publisher.publish(cmd)
    rate.sleep()

#end teleopmotion and exit node
cmd = Twist()
publisher.publish(cmd)
print "Motion Stopped; Teleop End"


