#!/usr/bin/env python

"""This script should publish a teal sphere in RViz at (1,2,0)_odom"""

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

#iniitalize
rospy.init_node('vizualize_rviz_sphere')
publisher = rospy.Publisher('/vizualization_marker', Marker, queue_size=10)

#message
marker = Marker(header=Header(stamp=rospy.Time(), frame_id="odom"),
                ns="vizualize_rviz_sphere", id=0, type=Marker.SPHERE, 
                action=Marker.ADD, pose=Pose(position=Point(1.0,2.0,0.0), 
                                             orientation=Quaternion(0.0,0.0,0.0,1.0)),
                scale=Vector3(0.5,0.5,0.5), color=ColorRGBA(0.0, 0.5, 0.5, 1.0))
print marker

#loop and update
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
    marker.header.stamp = rospy.Time()
    publisher.publish(marker)
    rate.sleep()

print "Marker Squashed"
