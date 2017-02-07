#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Vector3, Twist, Pose, Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan


class ObstacleAvoid(object):
    def __init__(self):
        rospy.init_node('obstacle_avoider')
        self.r = rospy.Rate(2)
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubViz = rospy.Publisher('markers', Marker, queue_size=10)
        rospy.Subscriber('stable_scan', LaserScan, self.processFunction)
        self.ranges = []
        self.cmd = Twist()
        self.viz = Marker()
        self.netForce = []

    def processFunction(self, msg):
        self.ranges = msg.ranges
        self.processForces()

    def processForces(self):
        forces = []
        for i in range(len(self.ranges)):
            if i <= 90 or i >= 270:
                if self.ranges[i]:
                    x = -self.ranges[i]*math.cos(math.radians(i))
                    y = -self.ranges[i]*math.sin(math.radians(i))
                    forces.append([x, y])

        netX = 0
        netY = 0
        for i in forces:
            netX += i[0]*100//1/100/len(forces)
            netY += i[1]*100//1/100/len(forces)

        netR = math.sqrt(netX**2 + netY**2)
        netTheta = math.atan(netY/netX)

        self.netForce = [netR, netTheta]
        print(self.netForce)



    def main(self):
        while not rospy.is_shutdown():
            self.pubCmd.publish(self.cmd)
            self.pubViz.publish(self.viz)


robot = ObstacleAvoid()
robot.main()
