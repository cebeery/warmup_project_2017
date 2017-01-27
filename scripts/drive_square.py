#!/usr/bin/env python

"""This script cmds Neato to move in a square via timed turns"""

import rospy
from geometry_msgs.msg import Twist
from time import time

class drive_square:
    """ Controls square driving behavior of neato"""

    def __init__(self):
        """inializes twist cmd and decision flags; sets time constants"""
        self.cmd = Twist()
        self.cornerTime = 3.04
        self.edgeTime = 2.05

        self.edges = 0
        self.inTurnState = True
        self.startTime = time()

    def corner(self):
        """updates twist cmd to turn; checks if turn is at end"""
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.5
        
        if (time() - self.startTime) > self.cornerTime:
            self.inTurnState = False
            print "Vertex Complete"
            self.startTime = time()
            
    def edge(self):
        """updates twist cmd to drive sraight; checks if edge is at end"""
        self.cmd.linear.x = 1.0
        self.cmd.angular.z = 0.0
        
        if (time() - self.startTime) > self.edgeTime:
            self.inTurnState = True
            self.edges += 1
            print "Edge Complete"
            self.startTime = time()

    def update(self):
        """Determine robot state and trigger cmd update""" 
        if self.inTurnState:
            self.corner()
        else:
            self.edge()
        return self.cmd


#iniitalize ros communication
rospy.init_node('drive_square')
publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

#make drive_square class instance
my_square = drive_square()

#loop key command updates
rate = rospy.Rate(20) 
while not rospy.is_shutdown() and my_square.edges<4: 
    cmd = my_square.update()
    publisher.publish(cmd)
    rate.sleep()

#end motion (if any) and exit node
publisher.publish(Twist())
print "Motion Ended"

if my_square.edges == 4:
    print "Square Completed"
else:
    print "Not Square"
