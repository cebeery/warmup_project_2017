#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Point32, Vector3
from sensor_msgs.msg import PointCloud, LaserScan
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
        rospy.Subscriber('stable_scan', LaserScan, self.processScan)
        rospy.Subscriber('odom', Odometry, self.processOdom)

        self.wallMarker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                                 scale=Vector3(0.5, 0.5, 0.5),
                                 color=ColorRGBA(0.8, 0.26, 0.25, 1.0),  #red
                                 type=Marker.SPHERE_LIST)

        #self.state = '**'
        #self.cmd = Twist()
        self.lastScan = []
        self.lastAngles = []
        self.currentScan = []
        self.currentAngles = []
        self.centerOfMass = Point32()
        self.motionThreshold = 0.05
        self.robotPosition = Point()
        self.kAngle = 0.017
        self.kDist = 0.2
        self.setPointDist = 0.5


    def processScan(self, msg):
        """
        self.lastScan = self.currentScan
        angles = msg.channels[1].values
        self.currentScan = []
        for i in range(len(angles)):
            if angles[i] < 45 or angles[i] > 315:
                if msg.points[i].x < 1.5:
                    self.currentScan.append(msg.points[i])
        """
        self.lastScan = self.currentScan
        self.lastAngles = self.currentAngles
        self.currentScan = []
        self.currentAngles = []
        for i in range(len(msg.ranges)):
            if i < 30 or i > 330:
                if msg.ranges[i] < 1.3 and not msg.ranges[i] == 0.0:
                    self.currentScan.append(msg.ranges[i])
                    self.currentAngles.append(i)


    def processOdom(self, msg):
        self.robotPosition = msg.pose.pose.position

    def COM(self, points):
        xSum = 0
        ySum = 0
        for i in range(len(points)):
            xSum += self.currentScan[i]*math.cos(math.radians(self.currentAngles[i]))
            ySum += self.currentScan[i]*math.sin(math.radians(self.currentAngles[i]))
        if len(points):
            return Point32(x=xSum/len(points), y=ySum/len(points))
        else:
            return Point(x=self.robotPosition.x+self.setPointDist, y=self.robotPosition.y)

    def distance(self, pointCom):

        return math.sqrt(pointCom.x**2 + pointCom.y**2)


    def calcAngle(self, pointCom):
        dX = pointCom.x
        dY = pointCom.y
        if not dX == 0 and not dY == 0:
            return math.degrees(math.atan(dY/dX))
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
            print(angle)
            self.pubViz.publish(self.wallMarker)
            self.cmd = Twist(linear=Vector3(x=xVel), angular=Vector3(z=zVal))
            self.pubCmd.publish(self.cmd)
            self.r.sleep()



robit = PersonFollow()
robit.main()
