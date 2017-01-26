#!/usr/bin/env python

"""This script cmds Neato to move in a square via timed turns"""

import rospy
from geometry_msgs.msg import Twist

class drive_square:
    """ Controls square driving behavior of neato"""

    def __init__(self):
        """inializes twist cmd and decision flags; sets time constants"""
        self.cmd = Twist()
        self.cornerTime = 1.0
        self.edgeTime = 2.0

        self.edges = 0
        self.inTurnState = True
        self.startTime = rospy.Time.now()

    def corner(self):
        """updates twist cmd to turn; checks if turn is at end"""
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 1.0
        
        if (rospy.Time.now() - self.startTime) > self.cornerTime:
            self.inTurnState = False
            print "Vertex Complete"
            self.startTime = rospy.Time.now()
            
    def edge(self):
        """updates twist cmd to drive sraight; checks if edge is at end"""
        self.cmd.linear.x = 0.4
        self.cmd.angular.z = 0.0
        
        if (rospy.Time.now() - self.startTime) > self.edgeTime:
            self.inTurnState = True
            self.edges += 1
            print "Edge Complete"
            self.startTime = rospy.Time.now()

    def update(self):
        """Determine robot state and trigger cmd update""" 
        if self.inTurningState:
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
rate = rospy.Rate(10) 
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
