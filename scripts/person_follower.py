#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist, Point, Point32, Vector3
from sensor_msgs.msg import PointCloud, LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA


class PersonFollow(object):
    def __init__(self):
        """ROS Node class for person following"""
        #Start up ROS Node communincation
        rospy.init_node('person_follower')
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubViz = rospy.Publisher('markers', Marker, queue_size=10)
        rospy.Subscriber('stable_scan', LaserScan, self.processScan)
        self.r = rospy.Rate(10)

        #pubscriber attributes
        self.currentScan = []
        self.currentAngles = []
        self.wallMarker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                                 scale=Vector3(0.5, 0.5, 0.5),
                                 color=ColorRGBA(0.8, 0.26, 0.25, 1.0),  #red
                                 type=Marker.SPHERE_LIST)

        #ctrl parameters 
        self.motionThreshold = 0.05
        self.kAngle = 0.017
        self.kDist = 0.2
        self.setPointDist = 0.5

        #status attribute
        self.robotPosition = Point()
        self.centerOfMass = Point32()

    def processScan(self, msg):
        """ Updates object angle and distance lists from bounded scan"""
        self.currentScan = []
        self.currentAngles = []
        for i in range(len(msg.ranges)):
            if i < 30 or i > 330:
                if msg.ranges[i] < 1.3 and not msg.ranges[i] == 0.0:
                    self.currentScan.append(msg.ranges[i])
                    self.currentAngles.append(i)

    def calcCOM(self, points):
        """calculates COM of seen objects, they exist """

        #sum object locations
        xSum = 0
        ySum = 0
        for i in range(len(points)):
            xSum += self.currentScan[i]*math.cos(math.radians(self.currentAngles[i]))
            ySum += self.currentScan[i]*math.sin(math.radians(self.currentAngles[i]))

        #return COM, else return ideal person location
        if len(points):
            return Point32(x=xSum/len(points), y=ySum/len(points))
        else:
            return Point(x=self.robotPosition.x+self.setPointDist, y=self.robotPosition.y)

    def calcDistance(self, pointCom):
        """calculate displacement from x,y """
        return math.sqrt(pointCom.x**2 + pointCom.y**2)


    def calcAngle(self, pointCom):
        """calculate angle to COM wrt base_link """
        dX = pointCom.x
        dY = pointCom.y
        if not dX == 0 and not dY == 0:
            return math.degrees(math.atan(dY/dX))
        else:
            return 0


    def main(self):
        """Run Loop -- calls function for current state, visualizes beliefs, publishes robot cmd"""
        while not rospy.is_shutdown():
            #Calculate COM characteristics
            centerOfMass = self.calcCOM(self.currentScan)
            dist = self.calcDistance(centerOfMass)
            angle = self.calcAngle(centerOfMass)

            #visualize COM
            self.wallMarker.points = []
            self.wallMarker.points.append(Point(x=centerOfMass.x, y=centerOfMass.y))

            #p-controller for speed and turning
            xVel = self.kDist*(dist-self.setPointDist)
            if math.fabs(dist-self.setPointDist) < 0.1:
                xVel = 0
            zVal = self.kAngle*(angle)
            if math.fabs(angle) < 2:
                zVal = 0
                        self.cmd = Twist(linear=Vector3(x=xVel), angular=Vector3(z=zVal))

            #publish visualization and cmds
            self.pubViz.publish(self.wallMarker)
            self.pubCmd.publish(self.cmd)
            self.r.sleep()


#Create and run instance 
robit = PersonFollow()
robit.main()
