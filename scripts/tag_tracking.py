#! /usr/bin/env python3

import rospy
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import ApriltagPoseStamped
from geometry_msgs.msg import PoseArray, Pose
import tf2_ros

import numpy as np

class TagTracking:
    def __init__(self):
        rospy.init_node('tag_tracking')

        self.world_tf_set = False
        self.o2p = None
        self.dfc2base = None

        self.mav_name = rospy.get_param('~mav_name', 'dragonfly21')
        self.odom_sub = rospy.Subscriber('{}/quadrotor_ukf/control_odom'.format(self.mav_name), Odometry, self.odom_cb)
        self.odom_tag_sub = rospy.Subscriber('{}/odom_tag'.format(self.mav_name), Odometry, self.odom_tag_cb)
        self.world_odom_pub = rospy.Publisher('{}/world_odom'.format(self.mav_name), Odometry, queue_size=10)
        self.tag_pose_sub = rospy.Subscriber('{}/dfc/apriltag_pose_estimator/apriltag_poses'.format(self.mav_name), ApriltagPoseStamped, self.tag_pose_cb)
        self.world_tag_pub = rospy.Publisher('{}/world_apriltag_poses'.format(self.mav_name), ApriltagPoseStamped, queue_size=10)
        self.dfc_frame_id = self.mav_name + '/dfc'
        self.base_frame_id = self.mav_name + '/base_link'
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo(f"mav_name: {self.mav_name}")
        rospy.loginfo("subscribe to odom: " + '{}/quadrotor_ukf/control_odom'.format(self.mav_name))
        rospy.loginfo("subscribe to odom_tag: " + '{}/odom_tag'.format(self.mav_name))
        rospy.loginfo("subscribe to tag_pose: " + '{}/dfc/apriltag_pose_estimator/apriltag_poses'.format(self.mav_name))
        rospy.loginfo('Tag Tracking Initialized')


    def odom_tag_cb(self, msg):
        self.odom_tag = msg
        if not self.world_tf_set and self.odom_pos is not None:
            self.b2w = np.eye(4)
            self.b2w[:3, :3] = R.from_quat([msg.pose.pose.orientation.x, 
                                            msg.pose.pose.orientation.y, 
                                            msg.pose.pose.orientation.z, 
                                            msg.pose.pose.orientation.w]).as_matrix()
            self.b2w[:3, 3] = np.array([msg.pose.pose.position.x, 
                                        msg.pose.pose.position.y, 
                                        msg.pose.pose.position.z])
            self.o2w = self.b2w @ np.linalg.inv(self.b2o)
            self.world_tf_set = True
            rospy.loginfo('World TF set!')


    def odom_cb(self, msg):
        self.b2o = np.eye(4)
        self.odom_pos = np.array([msg.pose.pose.position.x, 
                                  msg.pose.pose.position.y, 
                                  msg.pose.pose.position.z])
        self.odom_quat = np.array([msg.pose.pose.orientation.x, 
                                   msg.pose.pose.orientation.y, 
                                   msg.pose.pose.orientation.z, 
                                   msg.pose.pose.orientation.w])
        self.b2o[:3, :3] = R.from_quat(self.odom_quat).as_matrix()
        self.b2o[:3, 3] = self.odom_pos
        if not self.world_tf_set:
            rospy.logwarn('World TF not set!!!')
            return
        self.world_pose = np.eye(4)
        self.world_pose = self.o2w @ self.b2o
        # Compose odom message 
        world_odom = Odometry()
        world_odom.header.stamp = rospy.Time.now()
        world_odom.header.frame_id = 'global_origin'
        world_odom.pose.pose.position.x = self.world_pose[0, 3]
        world_odom.pose.pose.position.y = self.world_pose[1, 3]
        world_odom.pose.pose.position.z = self.world_pose[2, 3]
        world_odom.pose.pose.orientation.x, world_odom.pose.pose.orientation.y, world_odom.pose.pose.orientation.z, world_odom.pose.pose.orientation.w = R.from_matrix(self.world_pose[:3, :3]).as_quat()
        self.world_odom_pub.publish(world_odom)
    

    def lookup_dfc_to_base_link(self):
        if self.dfc2base is None:
            # It should contain at least means3D and radius
            # lookup the TF between map frame and the world frame
            try:
                # Lookup the static transform
                source_frame = self.base_frame_id
                target_frame = self.dfc_frame_id
                transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
                # Print out the transform details
                rospy.loginfo(f"Transform from {source_frame} to {target_frame}:")
                rospy.loginfo(f"Translation: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
                rospy.loginfo(f"Rotation: {transform.transform.rotation.x}, {transform.transform.rotation.y}, {transform.transform.rotation.z}, {transform.transform.rotation.w}")
            except tf2_ros.LookupException as e:
                rospy.logerr(f"Transform lookup failed: {e}")
                return
            except tf2_ros.ConnectivityException as e:
                rospy.logerr(f"Transform connectivity issue: {e}")
                return
            except tf2_ros.ExtrapolationException as e:
                rospy.logerr(f"Transform extrapolation issue: {e}")
                return
            self.dfc2base = np.eye(4)
            self.dfc2base[:3, :3] = R.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]).as_matrix()
            self.dfc2base[:3, 3] = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
        return

    def tag_pose_cb(self, msg):
        # read tag poses, they are in dfc frame. apply the static tf betweet dfc and base_link.
        # Lookup transform
        if not self.world_tf_set:
            rospy.logwarn('[tag pose cb] World TF not set!!!')
            return
        self.lookup_dfc_to_base_link()
        world_tag_msg = ApriltagPoseStamped()
        tag_pose_array = PoseArray()
        for i in range(len(msg.apriltags)):
            tag_pose_dfc = np.eye(4)
            tag_pose_dfc[:3, :3] = R.from_quat([msg.posearray.poses[i].orientation.x,
                                                msg.posearray.poses[i].orientation.y,
                                                msg.posearray.poses[i].orientation.z,
                                                msg.posearray.poses[i].orientation.w]).as_matrix()
            tag_pose_dfc[:3, 3] = np.array([msg.posearray.poses[i].position.x,
                                            msg.posearray.poses[i].position.y,
                                            msg.posearray.poses[i].position.z])
            tag_pose_base = self.dfc2base @ tag_pose_dfc
            # Then find tag pose in world frame
            world_pose = np.eye(4)
            world_pose = self.world_pose @ tag_pose_base
            tag_pose = Pose()
            tag_pose.position.x = world_pose[0, 3]
            tag_pose.position.y = world_pose[1, 3]
            tag_pose.position.z = world_pose[2, 3]
            tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w = R.from_matrix(world_pose[:3, :3]).as_quat()
            tag_pose_array.poses.append(tag_pose)

        # Compose world tag message
        world_tag_msg.header.stamp = rospy.Time.now()
        world_tag_msg.header.frame_id = 'global_origin'
        world_tag_msg.apriltags = msg.apriltags
        world_tag_msg.posearray = tag_pose_array
        self.world_tag_pub.publish(world_tag_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    tag_tracking = TagTracking()
    tag_tracking.run()