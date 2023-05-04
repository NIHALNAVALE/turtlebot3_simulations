#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import roslib
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from Perception.msg import HumanPose
from scipy.spatial.transform import Rotation


class idc:
    def __init__(self) -> None:
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # self.robot_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.client.wait_for_server()
        # map_type = rospy.get_param('map_type', 'house')
        self.map_type = 'house'
        self.maps = dict()
        self.locations = dict()
        self.distance = 0.0
        # 'value' is distance you want the robot to be from the human
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_x_q = 0.0
        self.robot_y_q = 0.0 
        self.robot_z_q = 0.0
        self.robot_w_q = 0.0

        self.human_x = 0.0
        self.human_y = 0.0
        self.human_z = 0.0
        self.human_x_q = 0.0
        self.human_y_q = 0.0
        self.human_z_q = 0.0
        self.human_w_q = 0.0

        self.rel_x = 0.0
        self.rel_y = 0.0
        self.rel_z = 0.0

        self.rel_x_q = 0.0
        self.rel_y_q = 0.0
        self.rel_z_q = 0.0
        self.rel_w_q = 0.0

        self.theta = 0.0
        self.ang = 0.0

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.quaternion_list = []
        self.z = 0.0
        self.rot_z = 0.0
        self.rot_quat = 0.0


    # odometry of robot
    def robot_odometry_client(self,data):
        # print("............ robot callback is working .............")

        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation

        # Extract the x, y, and z coordinates from the position
        self.robot_x = self.position.x
        self.robot_y = self.position.y
        self.robot_z = self.position.z

        # Extract the quaternion components from the orientation
        self.robot_x_q = self.orientation.x
        self.robot_y_q = self.orientation.y
        self.robot_z_q = self.orientation.z
        self.robot_w_q = self.orientation.w

    # odometry of human
    def human_odometry_client(self, data):
        # print("............ human callback is working .............")
        self.human_x = data.depth
        self.human_y = data.bb_x - data.frame_x
        self.human_z = 0.0

        self.human_x_q = 0.0
        self.human_y_q = 0.0
        self.human_z_q = 0.0
        self.human_w_q = 0.0

        self.movebase_client() # calls movebase client only when human is detected

    # movebase
    def movebase_client(self):

        # for i in range(1,10):
        # 	locations['Position'+str(i)] = Pose(Point(i/2, 0.5, 0.000), Quaternion(0.000, 0.000, 0.0, 0.001))
        # for key, value in locations.items():
        # 	print(key, ":", value)
        
        # getting ang for quaternion

        # getting theta for relative distance
        if self.human_x == 0:
            if self.human_y > 0:
                self.theta = 0
                self.ang = 0
            elif self.human_y < 0:
                self.theta = math.pi
                self.ang = math.pi
        else:
            self.theta = math.atan(self.human_y / self.human_x)
            self.ang = math.atan(self.human_y/self.human_x)
        # self.theta = math.atan((self.human_y)/(self.human_x))
        # print("theta :", self.theta)

        # human with respect to robot, rounded to 2 decimal places
        # self.rel_x = round(self.human_x + self.robot_x - self.distance, 2)
        self.rel_x = round((self.human_x - self.distance)* math.cos(self.theta) ,2)
        # self.rel_x = round(self.human_x - self.value, 2)
        # self.rel_y = round(self.human_y, 2)
        self.rel_y = round((self.human_x - self.distance)* math.sin(self.theta) ,2)
        self.rel_z = round(self.human_z - self.robot_z, 2)

        # rel_x_q = round(self.human_x_q - self.robot_x_q, 2)
        self.rel_x_q = round(self.robot_x_q, 2)
        self.rel_y_q = round(self.robot_y_q, 2)
        self.rel_z_q = round(self.robot_z_q, 2)
        self.rel_w_q = round(self.robot_w_q, 2)

        # rotation euler for theta
        self.quaternion_list = []
        # self.rot_z = Rotation.from_euler('xyz', [0, 0, self.theta], degrees=False)
        self.rot_z = Rotation.from_euler('xyz', [0, 0, self.ang], degrees=False)
        self.rot_quat = self.rot_z.as_quat()
        self.quaternion_list = self.rot_quat
        print("quaternion list :", self.quaternion_list)

        self.locations['Human wrt Robot'] = Pose(Point(self.rel_x, self.rel_y, self.rel_z), Quaternion(self.quaternion_list[0], self.quaternion_list[1], self.quaternion_list[2], self.quaternion_list[3]))
        # self.locations['Human wrt Robot'] = Pose(Point(self.rel_x, self.rel_y, self.rel_z), Quaternion(0.0, 0.0, 0.0, 0.01))
        
        # # test locations of rooms
        
        # self.locations['Human1'] = Pose(Point(1.0, 2.0, 0.000), Quaternion(0.000, 0.000, 0.75, 0.66))
        # self.locations['Human2'] = Pose(Point(1.0, -2.0, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))
        # self.locations['Human3'] = Pose(Point(1.0, 0.5, 0.000), Quaternion(0.000, 0.000, 0.999, 0.000))
        # self.locations['Human4'] = Pose(Point(1.0, 0.0, 0.000), Quaternion(0.000, 0.000, 0.999, 0.000))
        # self.locations['Human5'] = Pose(Point(1.0, 0.5, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))

        # Transforming the frames
        transform = self.tf_buffer.lookup_transform('map',
                                            # source frame:
                                            'base_link',
                                            # get the tf at the time the pose was valid
                                            rospy.Time.now(),
                                            # wait for at most 1 second for transform, otherwise throw
                                            rospy.Duration(5.0))

        # rospy.init_node('transform_pose_node')
        # tf_buffer = tf.Buffer()
        # tf_listener = tf2_ros.TransformListener(tf_buffer)
        # Wait for the transform to be available
        # tf_buffer.can_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
        # Create a PoseStamped message in the base_link frame
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = self.rel_x
        pose_msg.pose.position.y = self.rel_y
        pose_msg.pose.position.z = self.rel_z
        pose_msg.pose.orientation = Quaternion(*self.quaternion_list)    

        print("pose message :", pose_msg)
        # Transform the pose to the map frame
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
        # transformed_pose = tf_buffer.transformPose('map', pose_msg)
        # Print the transformed pose
        rospy.loginfo('Transformed pose: {}'.format(transformed_pose))

        # end of transforming frames

        self.locations['Human wrt Robot'] = transformed_pose.pose


        self.maps['house'] = self.locations
        # print('entered movebase client')
        # Looping in goal location sequence
        for self.location in self.maps[self.map_type].keys():  
            print("goal location x and y :", self.rel_x, self.rel_y)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            goal.target_pose.pose = self.maps[self.map_type][self.location]
            self.client.send_goal(goal)
            wait = self.client.wait_for_result()
            if not wait:
                rospy.logerr("Action server down")
            else:
                print("Reached " + self.location + " Goal") 
        return 1

if __name__ == '__main__':
    rospy.init_node('movebaseClient')

    client = idc()
    rospy.Subscriber('/odom', Odometry, client.robot_odometry_client)
    rospy.Subscriber('/Human_Pose', HumanPose, client.human_odometry_client)

    # Set the loop rate to 10 Hz (i.e., the loop will execute 10 times per second)
    loop_rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # Perform your node's operations here
        # ...
        
        loop_rate.sleep()  # Sleep for the remaining time to achieve the specified spin rate