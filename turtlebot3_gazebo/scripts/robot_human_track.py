#!/usr/bin/env python3

import rospy
import roslib
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

# distance you want the robot to be from the human
value = 0.0

def odometry_client(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    # Extract the x, y, and z coordinates from the position
    roobt_x = position.x
    robot_y = position.y
    robot_z = position.z

    # Extract the quaternion components from the orientation
    robot_x_q = orientation.x
    robot_y_q = orientation.y
    robot_z_q = orientation.z
    robot_w_q = orientation.w


# movebase
def movebase_client():

	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	# map_type = rospy.get_param('map_type', 'house')
	map_type = 'house'
	
	maps = dict()
	locations = dict()

	# for i in range(1,10):
	# 	locations['Position'+str(i)] = Pose(Point(i/2, 0.5, 0.000), Quaternion(0.000, 0.000, 0.0, 0.001))
	# for key, value in locations.items():
	# 	print(key, ":", value)
	
    # # test locations of rooms
	# locations['Human'] = Pose(Point(1.0, 2.5, 0.000), Quaternion(0.000, 0.000, 0.75, 0.66))
	# locations['Top_right_room'] = Pose(Point(6.221, 0.633, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))
	# locations['Mid_left_room'] = Pose(Point(3.517, 4.541, 0.000), Quaternion(0.000, 0.000, 0.999, 0.000))
	# locations['Bottom_room'] = Pose(Point(-4.058, 3.444, 0.000), Quaternion(0.000, 0.000, 0.999, 0.000))
	# locations['Bottom_room_right'] = Pose(Point(-6.279, 2.025, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))
	
    # human with respect to robot, rounded to 2 decimal places
    rel_x = round(human.x - robot_x, 2)
    # rel_x = round(human.x - value, 2)
    rel_y = round(human.y - robot_y, 2)
    rel_z = round(human.z - robot_z, 2)

    rel_x_q = round(human.x_q - robot_x_q, 2)
    rel_y_q = round(human.y_q - robot_y_q, 2)
    rel_z_q = round(human.z_q - robot_z_q, 2)
    rel_w_q = round(human.w_q - robot_w_q, 2)

    locations['Human wrt Robot'] = Pose(Point(rel_x, rel_y, rel_z), Quaternion(rel_x_q, rel_y_q, rel_z_q, rel_w_q))

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
		rospy.init_node('robot_odometry', anonymous=True) #make node 
        rospy.Subscriber('/odom', Odometry, odometry_client)
	
        rospy.init_node('movebaseClient')
		result = movebase_client()
		if result:
		    rospy.loginfo("All Goals executed")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation DONE ")