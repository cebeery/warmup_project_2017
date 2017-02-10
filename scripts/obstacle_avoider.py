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
        self.scanRange = 45
        self.scanThrow = 2 #*** arbitray
        self.kAngular = 0.3
        self.kLinear = 0.2

        
	#pubscriber storage attributes
        self.ranges = []
        self.cmd = Twist()
        self.cmd.linear.x = 0
        self.viz = Marker()

        #current status attributes
        self.netForce = [1.0,0.0]
	self.state = 'passive'

    def processScan(self, msg):
        """updates stored laser scan and calls conversion to force"""
        self.ranges = msg.ranges
        self.processForces()

    def processForces(self):
        """creates a repelent force based on weight average of laser scan"""     
        
        #converts laser scan points in boundry area to repellent forces inversely proportional to distance
        forces = []
        for i in range(len(self.ranges)): 
            if i <= self.scanRange or i >= 360 - self.scanRange: 
                if self.ranges[i] and self.ranges[i] < self.scanThrow: 
                    x = math.cos(math.radians(i))/self.ranges[i]
                    y = math.sin(math.radians(i))/self.ranges[i]
                    forces.append([x, y])

        #determine averaged net repellent force, if any
        if not forces:
            self.netForces = [[0, 0]]
            print "Nothing to See"
            netR = 0
            netTheta = 0
        else: 
            netX = 0
            netY = 0
            for i in forces:
                netX += i[0]/len(forces)
                netY += i[1]/len(forces)
                
	    #convert to polar coordinates for use with angular p-control
            netTheta = math.atan(netY/netX)
            netR = -math.sqrt(netX**2 + netY**2)
            self.netForce = [netR, netTheta]
            #print("{0:.2f}".format(netR), "{0:.2f}".format(netTheta)) #print 2 decimals of net force

    def setCmdPassive(self):
        """State using reactive propotional angular and linear speed when objects are nearish"""
        self.cmd.angular.z = self.netForce[0] * self.kAngular * math.copysign(1,self.netForce[1])
        print('Force: ' + str(self.netForce[0]))
        if math.fabs(self.netForce[0]) < 0.1:
            self.cmd.angular.z = 0
        self.cmd.linear.x = self.netForce[1]*self.kLinear*math.copysign(1, self.netForce[1])
        #print("Turn" + str(self.cmd.angular.z))

    def setCmdActive(self):
        """State using point turns to move away from very close objects"""
        self.cmd.angular.z = 0.2
        self.cmd.linear.x = 0.0

    def setCmdForward(self):
        """State using only a set forward linear motion when no objects nearish"""
        self.cmd.angular.z = 0.0
        self.cmd.linear.x = 0.2

    def checkState(self):
	if self.netForce[0] < -1.5: #****add second check of object immediately in from of it 
            self.state = 'active'
        #elif self.netForce[0] > -0.2: # ****currently arbitary
        #    self.state = 'forward'
        #else:
            self.state = 'passive'

        print("State")

    def main(self):
        """ Run Loop -- currently just allows CB to run"""
        while not rospy.is_shutdown():
            self.checkState()
            if self.state == 'passive':
                self.setCmdPassive()
            elif self.state == 'active':
                self.setCmdActive()
            #elif self.state == 'forward':
            #    self.setCmdForward()
            else:
                print("State Unknown")

            self.pubCmd.publish(self.cmd)
	    self.r.sleep()

#add vizualizer


# Run instance
robit = ObstacleAvoid()
robit.main()
