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

def main():
    rospy.init_node('nav_test', anonymous=False)

    global lastFifty
    lastFifty = [0] * 50
    global detection_index
    detection_index = 0
    global averageProb
    averageProb = 0.0
    
    # Create a list to hold the waypoint poses
    # Append each of the waypoints to the list.  
    # Each waypoint is a pose consisting of a position and orientation in the map frame.
    global waypoints
    waypoints = createWaypoints()       
    
    # Initialize the visualization markers for RViz
    init_markers()
    
    # Set a visualization marker at each waypoint        
    for waypoint in waypoints:           
        p = Point()
        p = waypoint.position
        markers.points.append(p)
        
    # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    # Subscribe to the turtlebot's scan topic
    scan_sub = rospy.Subscriber('/scan', LaserScan, setData)
    
    # Subscribe to the move_base action server
    global move_base
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    # Subscribe to the darknet_ros/bounding_boxes topic
    detection_data = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, getDetectionInfo)

    # Subscribe to the darknet_ros/found_object topic
    object_count = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, checkForObjectDetected)

    rospy.loginfo("Waiting for move_base action server...")
    
    # Wait 60 seconds for the action server to become available
    move_base.wait_for_server(rospy.Duration(60))
    
    rospy.loginfo("Connected to move base server")
    rospy.loginfo("Starting navigation test")

    global objectFound
    objectFound = False

    # Create 2 threads to navigate and detect objects simultaneously        
    object_detection_thread = threading.Thread(target=detect_object)
    navigation_thread = threading.Thread(target=navigate)

    # Start the threads
    object_detection_thread.start()
    navigation_thread.start()

    # Wait until the threads are done
    object_detection_thread.join()
    navigation_thread.join()

def getDetectionInfo(data):
    global xmin 
    xmin = data.bounding_boxes[0].xmin

    global xmax 
    xmax = data.bounding_boxes[0].xmax

    global probability 
    probability = data.bounding_boxes[0].probability

    global lastFifty
    global detection_index
    lastFifty[detection_index] = probability
    detection_index += 1
    detection_index %= 50

    global averageProb
    sumProbs = 0
    for i in lastFifty:
        sumProbs += i
    averageProb = sumProbs / 50

    global detection_class 
    detection_class = data.bounding_boxes[0].Class

def checkForObjectDetected(data):
    global count 
    count = data.count

def detect_object():
    global waypointID
    global count
    global move_base
    global cmd_vel_pub
    global objectFound
    global front
    waypointID = 0
    finalDone = False
    while not rospy.is_shutdown():
        while (count != 0 and averageProb > 0.5 and finalDone == False):
            rospy.loginfo("Object detected!")
            # update global variable to signal that an object is found
            objectFound = True

            # pause current waypoint goal
            currentWaypointID = waypointID
            move_base.cancel_goal()

            goalComplete = False
            while (not goalComplete and averageProb > 0.5 and finalDone == False):
                rospy.loginfo("Approaching object")
                # get direction to turn towards target
                turn_direction = targetDirection()

                stopMoving = Twist()
                stopMoving.linear.x = 0.0
                stopMoving.angular.z = 0.0

                turnRight = Twist()
                turnRight.linear.x = 0.0
                turnRight.angular.z = 0.5

                turnLeft = Twist()
                turnLeft.linear.x = 0.0
                turnLeft.angular.z = -0.5

                goStraight = Twist()
                goStraight.linear.x = 0.4
                goStraight.angular.z = 0.0

                if (turn_direction == 0):
                    objectFound = False
                elif (turn_direction == 1):
                    cmd_vel_pub.publish(turnRight)
                elif (turn_direction == 2):
                    cmd_vel_pub.publish(turnLeft)
                elif (turn_direction == 3):
                    cmd_vel_pub.publish(goStraight)

                if (front < 1.5):
                    rospy.loginfo("Object reached! SUCCESS!")
                    cmd_vel_pub.publish(stopMoving)
                    goalComplete = True
                    finalDone = True

                # Wait a little before publishing new command
                time.sleep(0.5)

            currentWaypointID -= 1
            waypointID = currentWaypointID

        # Check if object is detected 50 times per second
        time.sleep(0.5)

def targetDirection():
    global probability
    global detection_class
    # what confidence do we start heading towards the target
    confidence_limit = .6

    if probability > confidence_limit and detection_class == "bb8":
        #assuming 416 x 416 box
        bounding_box_center = (xmin + xmax) // 2
        if bounding_box_center < 175:
            return 1 # turn right
        if bounding_box_center > 241:
            return 2 # turn left
        else:
            return 3 # go straight   
    else:
        return 0

def navigate():
    global objectFound
    global marker_pub
    global waypoints
    global waypointID
    waypointID = 0
    currentWaypointID = 0

    # Initialize the waypoint goal
    goal = MoveBaseGoal()

    # Use the map frame to define goal poses
    goal.target_pose.header.frame_id = 'map'
    
    # Start navigating
    while not rospy.is_shutdown():
        if objectFound == False:
            #Update the marker display
            marker_pub.publish(markers)

            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()

            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = waypoints[waypointID]

            # Send the goal pose to the MoveBaseAction server
            move_base.send_goal(goal)
        
            # Allow 20 seconds to get there
            finished_within_time = move_base.wait_for_result(rospy.Duration(20)) 

            while (objectFound == True):
                time.sleep(0.1)

            # If we don't get there in time, abort the goal
            if not finished_within_time:
                move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")

            waypointID += 1
            waypointID %= len(waypoints)
                
def init_markers():
    global marker_pub
    global waypoints

    # Set up our waypoint markers
    marker_scale = 0.2
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
    
    # Define a marker publisher.
    marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
    
    # Initialize the marker points list.
    global markers
    markers = Marker()
    markers.ns = marker_ns
    markers.id = marker_id
    markers.type = Marker.CUBE_LIST
    markers.action = Marker.ADD
    markers.lifetime = rospy.Duration(marker_lifetime)
    markers.scale.x = marker_scale
    markers.scale.y = marker_scale
    markers.color.r = marker_color['r']
    markers.color.g = marker_color['g']
    markers.color.b = marker_color['b']
    markers.color.a = marker_color['a']
    
    markers.header.frame_id = 'odom'
    markers.header.stamp = rospy.Time.now()
    markers.points = list()

def shutdown():
    rospy.loginfo("Stopping the robot...")
    # Cancel any active goals
    move_base.cancel_goal()
    rospy.sleep(2)
    # Stop the robot
    cmd_vel_pub.publish(Twist())
    rospy.sleep(1)

def createWaypoints():
    robots = 1 # Number of robots
    searchArea = [-40, -40, 40, 40] # [startX, startY, endX, endY]
    robotIndex = 0 # robot ID in the swarm 0-delineated
    step = 5 # step size for the search grid

    offset = 1 / robots * (searchArea[2] - searchArea[0])
    robotSearchArea = [robotIndex * offset + searchArea[0], searchArea[1], (robotIndex + 1) * offset + searchArea[2], searchArea[3]]

    XOffset = robotSearchArea[0]
    YOffset = robotSearchArea[1]
    XBound = int(robotSearchArea[2])
    YBound = int(robotSearchArea[3])

    global waypoints
    waypoints = list()
    #return waypoints
    # Loop through the grid to search and create waypoints
    for xx in range(0, XBound, step):
        for yy in range(int(robotSearchArea[1]), YBound, step):
            # Calculate the coordinates of the waypoint
            waypointX = xx + XOffset
            waypointY = (yy if xx%2 == 0 else (YBound - yy))
            waypointZ = 0 # Always zero, turtlebots don't fly

            # Calculate the orientation of the waypoint
            waypointQuaternion = quaternion_from_euler(0, 0, 0 if xx%2 == 0 else pi, axes='sxyz')

            # Add the waypoint to the list
            waypoints.append(Pose(Point(waypointX, waypointY, waypointZ), Quaternion(*waypointQuaternion)))
    
    return waypoints
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")