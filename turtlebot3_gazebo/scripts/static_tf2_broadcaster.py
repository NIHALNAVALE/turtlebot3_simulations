# #!/usr/bin/env python

# import rospy
# import tf2_ros
# import geometry_msgs.msg

# if __name__ == '__main__':
#     rospy.init_node('static_tf2_broadcaster')

#     static_transformStamped = geometry_msgs.msg.TransformStamped()
#     static_transformStamped.header.stamp = rospy.Time.now()
#     static_transformStamped.header.frame_id = "map"
#     static_transformStamped.child_frame_id = "odom"
#     static_transformStamped.transform.translation.x = 0.0
#     static_transformStamped.transform.translation.y = 0.0
#     static_transformStamped.transform.translation.z = 0.0
#     quat = tf.transformations.quaternion_from_euler(0, 0, 0)
#     static_transformStamped.transform.rotation.x = quat[0]
#     static_transformStamped.transform.rotation.y = quat[1]
#     static_transformStamped.transform.rotation.z = quat[2]
#     static_transformStamped.transform.rotation.w = quat[3]

#     static_broadcaster = tf2_ros.StaticTransformBroadcaster()
#     static_broadcaster.sendTransform(static_transformStamped)

#     rospy.spin()
nihal@nihal-Ubuntu:~$ rostopic list
/amcl/parameter_descriptions
/amcl/parameter_updates
/amcl_pose
/camera/parameter_descriptions
/camera/parameter_updates
/camera/rgb/camera_info
/camera/rgb/image_raw
/camera/rgb/image_raw/compressed
/camera/rgb/image_raw/compressed/parameter_descriptions
/camera/rgb/image_raw/compressed/parameter_updates
/camera/rgb/image_raw/compressedDepth
/camera/rgb/image_raw/compressedDepth/parameter_descriptions
/camera/rgb/image_raw/compressedDepth/parameter_updates
/camera/rgb/image_raw/theora
/camera/rgb/image_raw/theora/parameter_descriptions
/camera/rgb/image_raw/theora/parameter_updates
/clicked_point
/clock
/cmd_vel
/diagnostics
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/initialpose
/joint_states
/map
/map_metadata
/map_updates
/move_base/DWAPlannerROS/cost_cloud
/move_base/DWAPlannerROS/global_plan
/move_base/DWAPlannerROS/local_plan
/move_base/DWAPlannerROS/parameter_descriptions
/move_base/DWAPlannerROS/parameter_updates
/move_base/DWAPlannerROS/trajectory_cloud
/move_base/NavfnROS/plan
/move_base/cancel
/move_base/current_goal
/move_base/feedback
/move_base/global_costmap/costmap
/move_base/global_costmap/costmap_updates
/move_base/global_costmap/footprint
/move_base/global_costmap/inflation_layer/parameter_descriptions
/move_base/global_costmap/inflation_layer/parameter_updates
/move_base/global_costmap/obstacle_layer/parameter_descriptions
/move_base/global_costmap/obstacle_layer/parameter_updates
/move_base/global_costmap/parameter_descriptions
/move_base/global_costmap/parameter_updates
/move_base/global_costmap/static_layer/parameter_descriptions
/move_base/global_costmap/static_layer/parameter_updates
/move_base/goal
/move_base/local_costmap/costmap
/move_base/local_costmap/costmap_updates
/move_base/local_costmap/footprint
/move_base/local_costmap/inflation_layer/parameter_descriptions
/move_base/local_costmap/inflation_layer/parameter_updates
/move_base/local_costmap/obstacle_layer/parameter_descriptions
/move_base/local_costmap/obstacle_layer/parameter_updates
/move_base/local_costmap/parameter_descriptions
/move_base/local_costmap/parameter_updates
/move_base/parameter_descriptions
/move_base/parameter_updates
/move_base/recovery_status
/move_base/result
/move_base/status
/move_base_simple/goal
/odom
/particlecloud
/rosout
/rosout_agg
/scan
/tf
/tf_static
/visualization_marker_array
