#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import math


class WallFollow(object):
    def __init__(self):
        rospy.init_node('wall_follower')
        self.state = 'finding'
        self.r = rospy.Rate(10)
        self.ranges = [0, 0, 0, 0, 0]
        self.cmd = Twist()
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubViz = rospy.Publisher('markers', Marker, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.processFunction)
        self.indexOffsetCCW = 90
        self.indexOffsetCW = 270
        self.angleOffset = 10
        self.angleToParallel = 0
        self.setpointDistance = 0.5
        self.distanceThreshold = 0.1
        self.kP = 0.01
        self.startTime = rospy.Time.now()
        self.wallMarker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                                 scale=Vector3(0.5, 0.5, 0.5),
                                 color=ColorRGBA(0.8, 0.26, 0.25, 1.0),  #red
                                 type=Marker.SPHERE_LIST)

    def processFunction(self, msg):
        self.ranges = [msg.ranges[self.angleOffset],  #Straight ahead
                       msg.ranges[360-self.angleOffset],  #Straight ahead
                       msg.ranges[270 + self.angleOffset],  #Right Side Wall
                       msg.ranges[270 - self.angleOffset],  #Right Side Wall
                       msg.ranges[0]]

    def lawOfCosines(self, a, b):
        x = self.ranges[a]
        y = self.ranges[b]
        angleZ = math.radians(2*self.angleOffset)

        z = math.sqrt(x**2 + y**2 - 2*x*y*math.cos(angleZ))
        try:
            angleX = math.acos((-(x**2) + y**2 + z**2)/(2*y*z))
            return math.degrees(angleX) + self.angleOffset
            print("in range")
        except:
            print("One range not in range")
            return 90

    def turnToFind(self, a, b):
        if self.ranges[a] and self.ranges[b]:
            self.angleToParallel = self.lawOfCosines(a, b)
            self.angleToParallel = self.angleToParallel-90
            self.cmd.angular.z = self.kP*-self.angleToParallel
            print(self.angleToParallel)
            if math.fabs(self.angleToParallel) < 1.0 and self.state == 'finding':
                self.state = 'setDistance'
            self.cmd.linear.x = 0
        elif self.ranges[a] or self.ranges[b]:
            self.cmd.angular.z = math.copysign(1, self.ranges[a] - self.ranges[b])*0.2
            self.cmd.linear.x = 0
            print("I am here")
            print(self.ranges[a], self.ranges[b])
        else:
            self.cmd.angular.z = 0
            self.cmd.linear.x = 0.6


    def setDistance(self):
        currentDist = self.ranges[4]
        error = self.setpointDistance - currentDist
        if currentDist == 0.0:
            self.cmd.linear.x = 0.3
        elif error <= self.distanceThreshold and error >= -self.distanceThreshold:
            self.cmd = Twist()
            self.startTime = rospy.Time.now()
            self.state = 'turnParallel'
        elif error > self.distanceThreshold:
            self.cmd.linear.x = -0.1
        elif error < self.distanceThreshold:
            self.cmd.linear.x = 0.1
        else:
            print(self.ranges[4]+ ' is current Distance from wall')
            print(error+ ' is current error with setpoint')

    def turnParallel(self):
        if rospy.Time.now()-self.startTime < rospy.Duration(3.2):
            self.cmd.angular.z = 0.5
        else:
            self.cmd =Twist()
            self.state = 'wallFollow'

    def wallFollow(self):
        currentDist = self.ranges[4]

        if currentDist == 0.0  or currentDist > 1:
            self.turnToFind(2, 3)
            self.cmd.linear.x = 0.2
        else:
            self.cmd.linear.x = 0
            self.state = 'finding'

    def markWall(self):

        angles = [self.angleOffset,  #  Straight ahead
                  -self.angleOffset,  #  Straight ahead
                  270 + self.angleOffset,  #  Right Side Wall
                  270 - self.angleOffset,  #  Right Side Wall
                  0]

        
        indexMarker = 0
        self.wallMarker.points = []

        for i in range(0, len(angles)-1):
            if not self.ranges[i] == 0.0:
                print('setting marker')
                angle = math.radians(angles[i])
                print angle 

                x = math.cos(angle)
                y = math.sin(angle)
                self.wallMarker.points.append(Point())
                self.wallMarker.points[indexMarker].x = self.ranges[i] * x
                self.wallMarker.points[indexMarker].y = self.ranges[i] * y

                indexMarker += 1
        self.wallMarker.header.stamp = rospy.Time.now()

    def main(self):


        while not rospy.is_shutdown():
            
            if self.state == 'finding':
                print 'Finding'
                self.turnToFind(0, 1)
            elif self.state == 'setDistance':
                print 'Setting Distance'
                self.setDistance()
            elif self.state == 'turnParallel':
                self.turnParallel()
                print 'Turning Parallel'
            elif self.state == 'wallFollow':
                print 'Wall Following'
                self.wallFollow()
            
            self.markWall()
            self.pubViz.publish(self.wallMarker)
            self.pubCmd.publish(self.cmd)
            self.r.sleep()


robit = WallFollow()
robit.main()
