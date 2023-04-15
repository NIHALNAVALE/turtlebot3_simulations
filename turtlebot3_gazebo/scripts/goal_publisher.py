#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Header

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('goal_publisher')

    # Create a publisher for the move_base/goal topic
    goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    # Create a move_base_msgs/MoveBaseActionGoal message
    msg = MoveBaseActionGoal()

    #  http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseActionGoal.html

    # Fill in the message fields
    msg.header = Header()
    msg.header.seq = 0
    msg.header.stamp.secs = 0
    msg.header.stamp.nsecs = 0
    msg.header.frame_id = ''

    msg.goal_id.stamp.secs = 0
    msg.goal_id.stamp.nsecs = 0
    msg.goal_id.id = ''

    msg.goal.target_pose.header.seq = 0
    msg.goal.target_pose.header.stamp.secs = 0
    msg.goal.target_pose.header.stamp.nsecs = 0
    msg.goal.target_pose.header.frame_id = 'map'

    msg.goal.target_pose.pose.position.x = 1.0
    msg.goal.target_pose.pose.position.y = 2.5
    msg.goal.target_pose.pose.position.z = 0.0

    msg.goal.target_pose.pose.orientation.x = 0.0
    msg.goal.target_pose.pose.orientation.y = 0.0
    msg.goal.target_pose.pose.orientation.z = 0.75
    msg.goal.target_pose.pose.orientation.w = 0.66

    # Publish the message once per second
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        goal_pub.publish(msg)
        rate.sleep()
