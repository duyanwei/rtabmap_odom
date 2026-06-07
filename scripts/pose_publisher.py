#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file camera_pose_publisher.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 06-13-2025
@version 1.0
@license Copyright (c) 2025
@desc None
'''

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import tf2_ros
import os
import numpy as np
from scipy.spatial.transform import Rotation


class PosePublisher:
    def __init__(self):
        rospy.init_node('pose_publisher')
        
        # Parameters
        self.pose_file = rospy.get_param('~pose_file', 'poses.txt')
        self.pose_freq = rospy.get_param("~pose_frequency", 30.0)
        self.timeout = 600.0  # seconds
        self.cam_sub = rospy.Subscriber("/left_cam_topic", Image, self.image_callback)
        self.visual_map_frame = rospy.get_param("~map_frame", "visual_map")
        # self.camera_frame = "cam0"
        self.camera_frame = rospy.get_param("~camera_frame", "cam0")
        self.invert_tf = rospy.get_param("~invert_tf", False)
        self.Tbc = np.eye(4)
        self.Tbc[:3, :3] = np.linalg.inv(Rotation.from_quat([0.500000, -0.500000, 0.500000, -0.500000]).as_matrix())
        self.Tcb = np.linalg.inv(self.Tbc)
        
        # Check if file exists
        if not os.path.exists(self.pose_file):
            rospy.logerr(f"Pose file {self.pose_file} not found!")
            rospy.signal_shutdown("File not found")
            return
        
        # TF broadcaster
        # self.tf_broadcaster = tf.TransformBroadcaster()
        self.br = tf2_ros.TransformBroadcaster()
        
        # Publisher
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10, latch=True)
        
        # Read poses
        self.poses = self.read_poses()
        self.current_pose_index = 0
        rospy.loginfo(f"Loaded {len(self.poses)} poses from {self.pose_file}")

        # Run
        rospy.spin()
        # self.run()


    def read_poses(self):
        """Read poses from text file (format: timestamp x y z qx qy qz qw)"""
        poses = np.loadtxt(self.pose_file, ndmin=2)
        # poses[:, 1], poses[:, 2] = poses[:, 2], -poses[:, 1]
        return poses
    
    def image_callback(self, msg):
        for index in range(self.current_pose_index, len(self.poses)):
            pose = self.poses[index]
            pose = self.__pose_transform(pose)
            pose[3] = 0.2
            diff = msg.header.stamp.to_sec() - pose[0]
            if np.abs(diff) < 1e-6:
                msg = Odometry()
                # Header
                msg.header.stamp = rospy.Time.from_sec(pose[0])
                msg.header.frame_id = self.visual_map_frame
                msg.child_frame_id = self.camera_frame
                
                # Position
                msg.pose.pose.position.x = pose[1]
                msg.pose.pose.position.y = pose[2]
                msg.pose.pose.position.z = pose[3]
                
                # Orientation (quaternion)
                msg.pose.pose.orientation.x = pose[4]
                msg.pose.pose.orientation.y = pose[5]
                msg.pose.pose.orientation.z = pose[6]
                msg.pose.pose.orientation.w = pose[7]
                
                # Publish and broadcast
                self.pub.publish(msg)

                t = TransformStamped()
                t.header.frame_id = self.visual_map_frame
                t.child_frame_id = self.camera_frame
                if self.invert_tf:
                    pose = self.__pose_inverse(pose)
                    t.header.frame_id = self.camera_frame
                    t.child_frame_id = self.visual_map_frame
                t.header.stamp = msg.header.stamp
                # print(t.header.stamp.to_sec())
                # - rospy.Duration.from_sec(0.5)
                t.transform.translation.x = pose[1]
                t.transform.translation.y = pose[2]
                t.transform.translation.z = pose[3]
                t.transform.rotation.x = pose[4]
                t.transform.rotation.y = pose[5]
                t.transform.rotation.z = pose[6]
                t.transform.rotation.w = pose[7]

                self.br.sendTransform(t)


                self.current_pose_index = index + 1
                break


    def __pose_inverse(self, pose):
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_quat(pose[4:]).as_matrix()
        mat[:3, 3] = pose[1:4]
        inv_mat = np.linalg.inv(mat)
        inv_pose = np.zeros(8)
        inv_pose[0] = pose[0]
        inv_pose[1:4] = inv_mat[:3, 3]
        inv_pose[4:] = Rotation.from_matrix(inv_mat[:3, :3]).as_quat()
        return inv_pose
    
    def __pose_transform(self, pose):
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_quat(pose[4:]).as_matrix()
        mat[:3, 3] = pose[1:4]
        inv_mat = self.Tcb @ mat @ self.Tbc
        inv_pose = np.zeros(8)
        inv_pose[0] = pose[0]
        inv_pose[1:4] = inv_mat[:3, 3]
        inv_pose[4:] = Rotation.from_matrix(inv_mat[:3, :3]).as_quat()
        return inv_pose


    def wait_for_subscriber(self):
        """Wait for subscribers with timeout"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(1.0)
        
        while not rospy.is_shutdown():
            if self.pub.get_num_connections() > 0:
                return True
            
            if (rospy.Time.now() - start_time).to_sec() >= self.timeout:
                return False
                
            rate.sleep()


    def run(self):

        # Wait for subscribers with timeout
        rospy.loginfo("wait subscriber")
        if not self.wait_for_subscriber():
            rospy.logwarn(f"No subscribers detected after {self.timeout} seconds. Shutting down.")
            return

        rate = rospy.Rate(self.pose_freq)
        rospy.loginfo("before while")
        while not rospy.is_shutdown() and self.current_pose_index < len(self.poses):
            """Publish current pose as TransformStamped"""
            pose = self.poses[self.current_pose_index]
            msg = Odometry()
            
            # Header
            msg.header.stamp = rospy.Time.from_sec(pose[0])
            msg.header.frame_id = self.visual_map_frame
            msg.child_frame_id = self.camera_frame
            
            # Position
            msg.pose.pose.position.x = pose[1]
            msg.pose.pose.position.y = pose[2]
            msg.pose.pose.position.z = pose[3]
            
            # Orientation (quaternion)
            msg.pose.pose.orientation.x = pose[4]
            msg.pose.pose.orientation.y = pose[5]
            msg.pose.pose.orientation.z = pose[6]
            msg.pose.pose.orientation.w = pose[7]
            
            # Publish and broadcast
            self.pub.publish(msg)
            # self.tf_broadcaster.sendTransform(
            #     (pose[0], pose[1], pose[2]),
            #     (pose[3], pose[4], pose[5], pose[6]),
            #     rospy.Time.now(),
            #     "world",
            #     "camera"
            # )
            
            rospy.loginfo(f"Published pose {self.current_pose_index + 1}/{len(self.poses)}")
            self.current_pose_index += 1
            rate.sleep()
        rospy.loginfo("ROS shutdown required")


if __name__ == '__main__':
    try:
        PosePublisher()
    except rospy.ROSInterruptException:
        pass
    print("Done")