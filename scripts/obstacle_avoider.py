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
        self.scanArea = 60
        self.kAngular = 0.3
        self.kLinear = 0.2
        self.state = 'passive'


	#cmd and sub storage attributes
        self.ranges = []
        self.cmd = Twist()
        self.cmd.linear.x = 0
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
                    x = math.cos(math.radians(i))/self.ranges[i]
                    y = math.sin(math.radians(i))/self.ranges[i]
                    forces.append([x, y])
        if not forces:
            self.netForces = [[0, 0]]
            print "Nothing to See"
            netR = 0
            netTheta = 0
        else:
            #add forces
            netX = 0
            netY = 0
            for i in forces:
                netX += i[0]/len(forces)
                netY += i[1]/len(forces)
                #print(netX, netY)

	               #convert to polar coordinates for use with angular p-control
            netTheta = math.atan(netY/netX)
            netR = -math.sqrt(netX**2 + netY**2)

        #constraint robot to never retreat to avoid reverse motion collisions
            if netR > 0.0:
                netR = 0.0

	#update net force attribute
            self.netForce = [netR, netTheta]
        #print("{0:.2f}".format(netR), "{0:.2f}".format(netTheta)) #print 2 decimals of net force

    def setCmdPassive(self):
        self.cmd.angular.z = self.netForce[0] * self.kAngular * math.copysign(1,self.netForce[1])
        print('Force: ' + str(self.netForce[0]))
        if math.fabs(self.netForce[0]) < 0.1:
            self.cmd.angular.z = 0
        self.cmd.linear.x = self.netForce[1]*self.kLinear*math.copysign(1, self.netForce[1])
        #print("Turn" + str(self.cmd.angular.z))

    def setCmdActive(self):
        self.cmd.angular.z = 0.1
        self.cmd.linear.x = 0[0]

    def checkState(self):
        print self.netForce[0]
        if self.netForce[0] < -1.5:
            self.state = 'active'
            print('act')
        else:
            self.state = 'passive'
            print('pass')

        print("State")

    def main(self):
        """ Run Loop -- currently just allows CB to run"""
        while not rospy.is_shutdown():
            self.checkState()
            if self.state == 'passive':
                self.setCmdPassive()
            if self.state == 'active':
                self.setCmdActive()
            self.pubCmd.publish(self.cmd)
            print(self.state)
	    self.r.sleep()


# Run instance
robit = ObstacleAvoid()
robit.main()
