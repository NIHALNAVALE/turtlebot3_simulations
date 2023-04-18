#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	# map_type = rospy.get_param('map_type', 'house')
	map_type = 'house'
	
	maps = dict()

	locations = dict()
	locations['Human'] = Pose(Point(1.0, 2.5, 0.000), Quaternion(0.000, 0.000, 0.75, 0.66))
	
	maps['house'] = locations

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	# Looping in goal location sequence
	for location in maps[map_type].keys():  
		
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		
		goal.target_pose.pose = maps[map_type][location]

		client.send_goal(goal)
		wait = client.wait_for_result()
		if not wait:
		    rospy.logerr("Action server down")
		else:
		    print("Reached " + location + " Goal") 
	return 1

if __name__ == '__main__':
	try:
		rospy.init_node('movebaseClient')
		result = movebase_client()
		if result:
		    rospy.loginfo("All Goals executed")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation DONE ")