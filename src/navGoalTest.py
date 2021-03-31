from geometry_msgs.msg import Pose, Point, Quaternion

goal = MoveBaseGoal()
goal.target_pose.pose = Pose(Point(1.5, -4.85, 0), Quaternion(0.0, 0.0, 0.6, 0.77))
goal.target_pose.header.frame_id = "map" 
goal.target_pose.header.stamp = rospy.Time() 
self.sac.send_goal(goal, active_cb=self.active_callback, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

def done_callback(self):
     rospy.loginfo("done callback")