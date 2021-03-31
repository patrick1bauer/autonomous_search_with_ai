#! /usr/bin/env python
# Patrick Bauer

import rospy
import time
import math
import actionlib

from std_msgs.msg import Empty
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock # Import clock capabilities
from sensor_msgs.msg import LaserScan # Import LIDAR capabilities
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler

class AutonomousTurtlebot(object):

    def __init__(self):
        
        self.ctrl_c = False

        self.sequence = 1

        time.

        # Rate at which to publish movements
        self.rate = rospy.Rate(30)

    def publishOnce(self, cmd):
        # This is because publishing in topics sometimes fails the first time you publish.
        # In continuos publishing systems there is no big deal but in systems that publish only
        # once it is very important.
        while not self.ctrl_c:
            connections = self.waypointPublisher.get_num_connections()
            if connections > 0:
                self.waypointPublisher.publish(cmd)
                break
            else:
                self.rate.sleep()

    def getClock(self, data):
        data.

    def getWaypoint(self):
        # get the next waypoint position and orientation.
        # TODO
        self.waypointMsg.header.seq = self.sequence
        self.sequence += 1 
        self.waypointMsg.header.stamp = rospy.Time()
        self.waypointMsg.header.frame_id = "map"
        self.waypointMsg.pose.position.x = 10.0
        self.waypointMsg.pose.position.y = 0.0
        self.waypointMsg.pose.position.z = 0.0
        self.waypointMsg.pose.orientation.x = 0.0
        self.waypointMsg.pose.orientation.y = 0.0
        self.waypointMsg.pose.orientation.z = 0.0
        self.waypointMsg.pose.orientation.w = 0.0

    def mission(self):
        self.waypointPublisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.waypointMsg = PoseStamped()

        self.getWaypoint()
        print("Waypoint: " + format(self.waypointMsg))
        # self.waypointPublisher.publish(self.waypointMsg)
        self.publishOnce(self.waypointMsg)

        """
        # Loop through waypoints until 
        self.targetFound = False
        while not targetFound:
            # Get next set of coordinates
            self.getWaypoint()

            # Go to location
            self.publishOnce(self.waypointMsg)

            # Wait until turtlebot is at waypoint before getting new waypoint
            self.atWaypoint = False
            while not atWaypoint:
                # Search for target
                # TODO
        """

if __name__ == '__main__':
    # Initialize publishing node
    rospy.init_node('target_search')

    # Create turtlebot object(s)
    turtlebot1 = AutonomousTurtlebot()

    # Create subscriber
    clock_sub = rospy.Subscriber('/clock', Clock, turtlebot1.getClock)

    # Run the missions
    try:
        turtlebot1.mission()
    except rospy.ROSInterruptException:
        pass
