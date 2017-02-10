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
        self.kp = 0.3
        
	
	#cmd and sub storage attributes
        self.ranges = []
        self.cmd = Twist()
        self.cmd.linear.x = 0.2
        self.viz = Marker()

        #current status attributes
        self.netForce = [1.0,0.0]

    def processScan(self, msg):
        """updates stored laser scan and calls conversion to force"""
        print("Scanning") 
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
            forces= [0,0]
            print "Nothing to See"

        #add forces 
        netX = 0
        netY = 0
        for i in forces:
            netX += i[0]/len(forces)
            netY += i[1]/len(forces)
        print(netX, netY)  

	#convert to polar coordinates for use with angular p-controller
	netTheta = math.atan(netY/netX)
        netR = -math.sqrt(netX**2 + netY**2)
        netTheta = math.atan(netY/netX)

        #constraint robot to never retreat to avoid reverse motion collisions
        if netR > 0.0:
           netR = 0.0

	#update net force attribute
        self.netForce = [netR, netTheta]
        print("{0:.2f}".format(netR), "{0:.2f}".format(netTheta)) #print 2 decimals of net force 

    def setCmd(self):
        self.cmd.angular.z = self.netForce[0] * self.kp * math.copysign(1,self.netForce[1])
        print("Turn" + str(self.cmd.angular.z))
 
    def main(self):
        """ Run Loop -- currently just allows CB to run"""
        while not rospy.is_shutdown():
            self.setCmd()
            self.pubCmd.publish(self.cmd)
	    self.r.sleep()

# Run instance
robit = ObstacleAvoid()
robit.main()
