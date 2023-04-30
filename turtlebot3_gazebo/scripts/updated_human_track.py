#!/usr/bin/env python3

import rospy
import actionlib
from scipy.spatial.transform import Rotation
from turtlebot3_gazebo.msg import HumanPose
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from controller import PIDController

class idc:
	def __init__(self) -> None:
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.robot_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.client.wait_for_server()
	
		# map_type = rospy.get_param('map_type', 'house')
		self.map_type = 'house'
	
		self.maps = dict()
		self.locations = dict()
		

	def callbackfnc(self,data):
		data1 = {}

		self.rot_error = data.bb_x - data.frame_x
		print("Rotation error: ",self.rot_error)
		z = 0.001*self.rot_error
		# Create a rotation object from Euler angles specifying axes of rotation
		rot = Rotation.from_euler('xyz', [0, 0, z], degrees=False)

		# Convert to quaternions and print
		rot_quat = rot.as_quat()

		data1['qauter'] = rot_quat
		data1['depth'] = data.depth
		# data1['depth'] = 0
		self.movebase_client(data1)




	def movebase_client(self,data1):
		x, y, z, w = data1['qauter']
		print ("depth and quater:	", data1['depth'], data1['qauter'])

		# if abs(self.rot_error) > 150:
		# 	x, y, z, w = data1['qauter']
		# 	r = Rotation.from_quat([x, y, z, w])
		# 	r_euler = r.as_euler('zyx')
		# 	self.msg = Twist()
		# 	self.msg.angular.z = r_euler[2]
		# 	self.msg
		# 	self.robot_pub.publish(self.msg)
		# 	print ("controller depth and quater:	", data1['depth'], data1['qauter'])
		# else:
		# 	x, y, z, w = data1['qauter']
		# 	print ("depth and quater:	", data1['depth'], data1['qauter'])
		# x,y,z,w = 0,0,0,0
		
		self.locations['Human'] = Pose(Point(data1['depth'], 0.0, 0.000), Quaternion(x=0, y=0, z=0, w=0.001))

		# for i in range(1,20):
		# 	self.locations['Position'+str(i)] = Pose(Point(data1['depth']/2, 0.5, 0.000), Quaternion(0.000, 0.000, 0.0, 0.001))
		# for key, value in self.locations.items():
		# 	print(key, ":", value)

		self.maps['house'] = self.locations
		

		# Looping in goal location sequence
		for location in self.maps[self.map_type].keys():
			
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			
			goal.target_pose.pose = self.maps[self.map_type][location]

			self.client.send_goal(goal)
			wait = self.client.wait_for_result()
			if not wait:
				rospy.logerr("Action server down")
			else:
				print("Reached " + location + " Goal") 
		return 1

if __name__ == '__main__':
	# try:
	# 	rospy.init_node('movebaseClient')

	# 	client = idc()
	# 	pose_sub = rospy.Subscriber('/Human_pose', HumanPose,client.callbackfnc)
	# 	result = client.movebase_client()
	# 	if result:
	# 	    rospy.loginfo("All Goals executed")
	# except rospy.ROSInterruptException:
	# 	rospy.loginfo("Navigation DONE ")

	rospy.init_node('movebaseClient')
	client = idc()
	pose_sub = rospy.Subscriber('/Human_pose', HumanPose,client.callbackfnc)
	
	rospy.spin()