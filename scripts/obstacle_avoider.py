#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Vector3, Twist, Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan


class ObstacleAvoid(object):
    """Creates ROS node that controls obstacle avoidance behavior of Neato"""

    def __init__(self):
        """Initialize ROS node and create class atrributes"""

        #Starts up ROS coummincation
        rospy.init_node('obstacle_avoider')
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubViz = rospy.Publisher('markers', Marker, queue_size=10)
        rospy.Subscriber('stable_scan', LaserScan, self.processScan)
        self.r = rospy.Rate(2)

        #control attribute
        self.scanArea = 90
	
	#cmd and sub storage attributes
        self.ranges = []
        self.cmd = Twist()
        self.viz = Marker()

        #current status attributes
        self.netForce = [1.0,0.0]

    def processScan(self, msg):
        """updates stored laser scan and calls conversion to force"""
        self.ranges = msg.ranges
        self.processForces()

    def processForces(self):
        """converts laser scan points in scan area to repellent forces inversely proportional to distance"""
        forces = []
        for i in range(len(self.ranges)):
            if i <= self.scanArea or i >= 360 - self.scanArea:
                if self.ranges[i]:
                    x = -math.cos(math.radians(i))/self.ranges[i]
                    y = -math.sin(math.radians(i))/self.ranges[i]
                    forces.append([x, y])

        #add forces 
        netX = 0
        netY = 0
        for i in forces:
            netX += i[0]
            netY += i[1]

	#convert to polar coordinates for use with angular p-controller
        netR = math.sqrt(netX**2 + netY**2)
        netTheta = math.atan(netY/netX)

        #constraint robot to never retreat to avoid reverse motion collisions
        if netR < 0.0:
           netR = 0.0

	#update net force attribute
        self.netForce = [netR, netTheta]
        print("{0:.2f}".format(netR), "{0:.2f}".format(netTheta)) #print 2 decimals of net force 



    def main(self):
        """ Run Loop -- currently just allows CB to run"""
        while not rospy.is_shutdown():
            #self.pubCmd.publish(self.cmd)
            #self.pubViz.publish(self.viz)
	    pass

# Run instance
robit = ObstacleAvoid()
robit.main()
