#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Point32, Vector3
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
import math


class PersonFollow(object):
    def __init__(self):
        rospy.init_node('person_follower')
        self.r = rospy.Rate(10)
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubViz = rospy.Publisher('markers', Marker, queue_size=10)
        rospy.Subscriber('projected_stable_scan', PointCloud, self.processScan)
        rospy.Subscriber('odom', Odometry, self.processOdom)

        self.wallMarker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='odom'),
                                 scale=Vector3(0.5, 0.5, 0.5),
                                 color=ColorRGBA(0.8, 0.26, 0.25, 1.0),  #red
                                 type=Marker.SPHERE_LIST)

        #self.state = '**'
        #self.cmd = Twist()
        self.lastScan = [Point32()]
        self.currentScan = [Point32()]
        self.centerOfMass = Point32()
        self.motionThreshold = 0.05
        self.robotPosition = Point()
        self.kAngle = -0.0005
        self.kDist = 0.3
        self.setPointDist = 0.5


    def processScan(self, msg):
        self.lastScan = self.currentScan
        angles = msg.channels[1].values
        self.currentScan = []
        for i in range(len(angles)):
            if angles[i] < 45 or angles[i] > 315:
                self.currentScan.append(msg.points[i])

    def processOdom(self, msg):
        self.robotPosition = msg.pose.pose.position

    def COM(self, points):
        xSum = 0
        ySum = 0
        for i in points:
            xSum += i.x
            ySum += i.y
        if len(points):
            return Point32(x=xSum/len(points), y=ySum/len(points))
        else:
            return Point(x=self.robotPosition.x+self.setPointDist, y=self.robotPosition.y)

    def distance(self, pointCom):
        dX = pointCom.x - self.robotPosition.x
        dY = pointCom.y - self.robotPosition.y

        return math.sqrt(dX**2 + dY**2)

    def calcAngle(self, pointCom):
        dX = pointCom.x - self.robotPosition.x
        dY = pointCom.y - self.robotPosition.y
        if not dX == 0 and not dY == 0:
            return math.degrees(math.atan(dX/dY))
        else:
            return 0


    def main(self):
        while not rospy.is_shutdown():

            #self.matchPoints()
            centerOfMass = self.COM(self.currentScan)
            dist = self.distance(centerOfMass)
            angle = self.calcAngle(centerOfMass)
            self.wallMarker.points = []
            self.wallMarker.points.append(Point(x=centerOfMass.x, y=centerOfMass.y))

            xVel = self.kDist*(dist-self.setPointDist)
            if math.fabs(dist-self.setPointDist) < 0.1:
                xVel = 0

            zVal = self.kAngle*(angle)
            if math.fabs(angle) < 2:
                zVal = 0
            print angle
            self.pubViz.publish(self.wallMarker)
            self.cmd = Twist(linear=Vector3(x=0), angular=Vector3(z=zVal))
            self.pubCmd.publish(self.cmd)
            self.r.sleep()



robit = PersonFollow()
robit.main()
