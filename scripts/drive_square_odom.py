#!/usr/bin/env python

"""This script cmds Neato to move in a square via odom values"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class OdomDriveSquareNode(object):
    """ Controls square driving behavior of neato"""

    def __init__(self):
        """initalizes twist cmd and decision flags; sets relative odom values """
        self.cmd = Twist()
        self.cornerAng= 0.75
        self.edgeLen = 1.0

        self.edges = 0
        self.inTurnState = True
        self.startOdom = {x:0.0,y:0.0,z:0.0}
        self.currentOdom = {x:0.0,y:0.0,z:0.0}

        rospy.init_node('drive_sqaure_odom')
        rospy.Subscriber('odom', Odometry, self.processOdom) 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def processOdom(self, m)
        """ still need to verify multiple frame and pose/twist*** """
        self.currentOdom.x = m.pose.position.x
        self.currentOdom.y = m.pose.position.y
        self.currentOdom.z = m.pose.orientation.z

    def distanceCorrect(self):
        """Check to see if linear distance traveled is sufficient"""
        x = self.startOdom.x-self.currentOdom.x
        y = self.startOdom.y-self.currentOdom.y
        dist = (x**2.0 + y*2.0)**(0.5)
        if dist < self.edgeLen:
            return False
        else:
            return True

    def angleCorrect(self):
        """Check to see if angular distance traveled is sufficient"""
        Pass ****

    def corner(self):
        """updates twist cmd to turn; checks if turn is at end"""
        if not angleCorrect():
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 1.0
        else:
            self.cmd = Twist()
            self.startOdom = self.currentOdom
            self.inTurnState = False
            print "Vertex Complete"
            
    def edge(self):
        """updates twist cmd to drive sraight; checks if edge is at end"""
        dist = check_dist()
        
        if dist < self.edgeLen:
            self.cmd.linear.x = 1.0
            self.cmd.angular.z = 0.0
        else:
            self.cmd = Twist()
            self.startOdom = self.currentOdom
            self.inTurnState = True
            self.edges += 1
            print "Edge Complete"

    def run(self):
        """Determine robot state and trigger cmd update"""
        while not rospy.is_shutdown() and self.edges<4:      
            if self.inTurnState:
                self.corner()
            else:
                self.edge()
        
            self.pub.publish(self.cmd)
            self.rate.sleep()

        #end motion (if any) and exit node
        self.pub.publish(Twist())
        print "Motion Ended"

        if self.edges == 4:
            print "Square Completed"
        else:
            print "Not Square"


#run odom square instance
my_square = OdomDriveSquareNode()
my.square.run()
