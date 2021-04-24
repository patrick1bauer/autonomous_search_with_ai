#!/usr/bin/env python

import time
import threading
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

# Create shared variables for threads
global objectFound
global count
global xmin
global xmax
global probability
global detection_class
global front
global markers
global move_base
global cmd_vel_pub
global marker_pub
global waypoints
global waypointID
global lastFifty
global detection_index
global averageProb

def setData(data):
    global front
    front = data.ranges[89]
    rospy.loginfo("Front: " + format(front))