#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA


class WallFollow(object):
    """ROS Node class for wall following"""
    def __init__(self):
        """Initializes ROS communication and sets class attributes"""
        #Start up ROS Node communincation
        rospy.init_node('wall_follower')
        self.pubCmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubViz = rospy.Publisher('markers', Marker, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.processScan)
        self.r = rospy.Rate(10)

        #robot status attributes
        self.state = 'findWall'
        self.startTime = rospy.Time.now()
        self.angleToParallel = 0

        #pubscriber attributes
        self.ranges = [0, 0, 0, 0, 0]
        self.cmd = Twist()
        self.wallMarker = Marker(header=Header(stamp=rospy.Time.now(), frame_id='base_link'),
                                 scale=Vector3(0.5, 0.5, 0.5),
                                 color=ColorRGBA(0.8, 0.26, 0.25, 1.0),  #red
                                 type=Marker.SPHERE_LIST)

        #ctrl parameters
        self.angleOffset = 10
        self.distanceThreshold = 0.1
        self.angleThreshold = 1.0
        self.kP = 0.01
        self.setpointDistance = 0.5

    def processScan(self, msg):
        """Update ranges attributes with magnetudes of preset angles of laser scan """
        self.ranges = [msg.ranges[self.angleOffset],        #CW Ahead
                       msg.ranges[360 - self.angleOffset],  #CCW Ahead
                       msg.ranges[270 + self.angleOffset],  #CW Right Side  
                       msg.ranges[270 - self.angleOffset],  #CCW Right Side 
                       msg.ranges[0]]                       #Directly Ahead

    def markWall(self):
        """Updates marker point list with currently seen points selected scan points """
        angles = [self.angleOffset,        #CW Ahead
                  -self.angleOffset,       #CCW Ahead
                  270 + self.angleOffset,  #CW Right Side
                  270 - self.angleOffset,  #CCW Right Side
                  0]                       #Directly Ahead

        indexMarker = 0
        self.wallMarker.points = []
        for i in range(0, len(angles)-1):
            if not self.ranges[i] == 0.0:  #for all seen points in selected angle set
                #convert to cartesian coordinates
                angle = math.radians(angles[i])
                x = math.cos(angle)
                y = math.sin(angle)
 
                #add points to list
                self.wallMarker.points.append(Point())
                self.wallMarker.points[indexMarker].x = self.ranges[i] * x
                self.wallMarker.points[indexMarker].y = self.ranges[i] * y
                
                #track number of seen points for indexing marker list
                indexMarker += 1

        #update marker header with current time
        self.wallMarker.header.stamp = rospy.Time.now()

    def lawOfCosines(self, a, b):
        """calculates selected characteristics of triangle formed by scan and wall"""
        x = self.ranges[a]
        y = self.ranges[b]
        angleZ = math.radians(2*self.angleOffset)

        #law of cosines -- determine length of wall between scan intersection
        z = math.sqrt(x**2 + y**2 - 2*x*y*math.cos(angleZ)) 

        try:
            #law of cosines -- Angle of corner across from x side
            angleX = math.acos((-(x**2) + y**2 + z**2)/(2*y*z))
        except:
            #print("One range not in range")
            angleX = 90 - self.angleOffset
        return z, angleX

    def calcAngle(self, a, b):
        """calculates robot's angle relative wall given two distances from wall"""
        z, angleX = self.lawOfCosines(a,b)
	return math.degrees(angleX) + self.angleOffset

    def calcHeight(self, a, b):
        """calculates distance of robot from wall along perdendicular to wall"""
        z, angleX = self.lawOfCosines(a,b)
        return math.sin(angleX)*y

    #
    # States
    #

    def turnToFind(self, a, b):
        """
	Both finding state and indep. function
        State: finds and turns toward wall; when aligned change to setDistance state
        Function: p-controller for turning to make used scan axii perpendicular to wall
        """
         
        if self.ranges[a] and self.ranges[b]: 
            #both points are seen --> p-controller on turns to make robot perpendicular to wall
            self.angleToParallel = self.calcAngle(a, b)-90
            self.cmd.angular.z = self.kP*-self.angleToParallel
            if math.fabs(self.angleToParallel) < self.angleThreshold and self.state == 'findWall':
                #switch state to setDistance if within angualr tolerances
                self.state = 'setDistance'
            self.cmd.linear.x = 0
        elif self.ranges[a] or self.ranges[b]:
            #one point seen --> turn towards point
            self.cmd.angular.z = math.copysign(1, self.ranges[a] - self.ranges[b])*0.2
            self.cmd.linear.x = 0
        else:
            #no wall seen --> drive foward
            self.cmd.angular.z = 0
            self.cmd.linear.x = 0.5

    def setDistance(self):
        """Moves robot forward/back to desired distance from wall; changes to turnParallel state on completion"""
        #create delta from desired postion
        currentDist = self.ranges[4]
        error = self.setpointDistance - currentDist

        #Bang-Bang controller 
        if currentDist == 0.0:
            #if distance error --> drive forward
            self.cmd.linear.x = 0.2
        elif error <= self.distanceThreshold and error >= -self.distanceThreshold:
            #if within allowable distance threshold --> change to turnParallel
            self.cmd = Twist()
            self.startTime = rospy.Time.now()
            self.state = 'turnParallel'
        elif error > self.distanceThreshold:
            #if too close to wall --> move backward
            self.cmd.linear.x = -0.1
        elif error < self.distanceThreshold:
            #if too far from wall --> move forward
            self.cmd.linear.x = 0.1
        else:
            #bad fairies --> print beliefs
            print(self.ranges[4]+ ' is current Distance from wall')
            print(error+ ' is current error with setpoint')

    def turnParallel(self):
        """ turns ccw until set duration completed, then switches to followWall""" 
        if rospy.Time.now()-self.startTime < rospy.Duration(3.2):
            self.cmd.angular.z = 0.5
        else:
            self.cmd =Twist()
            self.state = 'followWall'

    def turnPerpindicular(self):
        """ turns cw until set duration completed, then switches to findWall""" 
        if rospy.Time.now()-self.startTime < rospy.Duration(3.2):
            self.cmd.angular.z = -0.5
        else:
            self.cmd =Twist()
            self.state = 'findWall'

    def wallFollow(self):
        """
	attempts to stay parallel to wall by calling findWall's angular p-controller on robots -y direction
        State changes: 1) too far from wall --> change to correctDistance
                       2) object in front --> change to findWall 
        """

        #determine wall distance, if can
        if self.ranges[3]:
            h = self.calcHeight(2, 3)
        else:
            h = 0.0

        #change state or run state switchcase
        currentDist = self.ranges[4]
        if h and math.fabs(self.setpointDistance - h) > 0.2:
            # if too far from wall --> change to correctDistance state
            self.cmd.linear.x = 0
            self.cmd.angular.z = 0
            self.state == 'correctDistance'
            self.startTime = rospy.Time.now()
        elif currentDist == 0.0 or currentDist > 1:
            # if no object in front --> correct wall parallal as moving forward
            self.turnToFind(2, 3)
            self.cmd.linear.x = 0.2
        else:
            # if object in front --> change to findWall state to align with new wall/object
            self.cmd.linear.x = 0
            self.state = 'findWall'



    def main(self):
        """Run Loop -- calls function for current state, visualizes beliefs, publishes robot cmd"""
        while not rospy.is_shutdown():
            #run state
            if self.state == 'findWall':
                print 'Finding Wall'
                self.turnToFind(0, 1)
            elif self.state == 'setDistance':
                print 'Setting Distance to Wall'
                self.setDistance()
            elif self.state == 'turnParallel':
                self.turnParallel()
                print 'Turning Parallel to Wall'
            elif self.state == 'followWall':
                print 'Wall Following'
                self.wallFollow()
            elif self.state == 'correctDistance':
                print 'Correcting Distance to Wall'
                self.turnPerpindicular()

            # visualize wall and send robot cmd
            self.markWall()
            self.pubViz.publish(self.wallMarker)
            self.pubCmd.publish(self.cmd)
            self.r.sleep()

#Create and run instance 
robit = WallFollow()
robit.main()
