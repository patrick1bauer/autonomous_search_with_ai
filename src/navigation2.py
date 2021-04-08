import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class TurtlebotCheckpoints():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        # How big is the square we want the robot to navigate?
        square_size = rospy.get_param("~square_size", 1.0) # meters
        
        # Create a list to hold the waypoint poses
        waypoints = list()
        
        # Create the waypoints and add them to the list
        self.createWaypoints()

        # Initialize the visualization markers for RViz
        self.init_markers()
        
        # Set a visualization marker at each waypoint        
        for waypoint in waypoints:           
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)
            
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 10 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(10))
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting turtlebot navigation")
        
        # Initialize a counter to track waypoints
        i = 0

        # Go through every waypoint and 
        for waypoint in waypoints:
            # Update the marker display
            self.marker_pub.publish(self.markers)

            # Initialize the waypoint goal
            goal = MoveBaseGoal()

            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'map'

            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()

            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = waypoints[i]

            # Start the robot moving toward the goal
            self.move_turtlebot(goal)

            # Increment i to get the next waypoint on the next loop.
            i = i + 1

        # Return turtlebot to start point

        
    def move_turtlebot(self, goal):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)
            
            # Allow 30 seconds to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(30)) 
            
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    
    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def createWaypoints(self):

        drones = 5 # Number of drones
        searchArea = [-38, -38, 102, 38] #[startX, startY, endX, endY]
        droneIndex = 0 # Drone ID in the swarm 0-delineated
        step = 5 # step size for the search grid

        offset = 1 / drones * (searchArea[2] - searchArea[0])
        droneSearchArea = [droneIndex * offset + startX, startY, (droneindex + 1) * offset + startX, endY]

        XOffset = droneSearchArea[0]
        YOffset = droneSearchArea[1]
        XBound = droneSearchArea[2]
        YBound = droneSearchArea[3]

        # Loop through the grid to search and create waypoints
        for x in range(0, XBound, step):
            for y in range(0, YBound, step):
                # Calculate the coordinates of the waypoint
                waypointX = x + XOffset
                waypointY = (y if x%2 == 0 else (YBound - y))
                waypointZ = 0 # Always zero, turtlebots don't fly

                # Calculate the orientation of the waypoint
                waypointQuaternion = tf.transformations.quaternion_from_euler(0, 0 if x%2 == 0 else math.pi, 0)

                # Add the waypoint to the list
                self.waypoints.append(Pose(Point(waypointX, waypointY, waypointZ), waypointQuaternion))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        TurtlebotCheckpoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")