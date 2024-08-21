#! /usr/bin/env python

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from apriltag_msgs.msg import ApriltagPoseStamped

class TagPosePub:
    def __init__(self):
        rospy.init_node('tag_pose_pub')
        self.tag_pub = rospy.Publisher('tag_poses', MarkerArray, queue_size=10)
        self.tag_sub = rospy.Subscriber('/dragonfly21/dfc/apriltag_pose_estimator/apriltag_poses', ApriltagPoseStamped, self.tag_cb)
        rospy.loginfo('Tag Pose Publisher Initialized')
        
    def tag_cb(self, data):
        marker_array = MarkerArray()
        for i in range(len(data.apriltags)):
            marker = Marker()
            marker.header.frame_id = data.posearray.header.frame_id
            marker.header.stamp = data.posearray.header.stamp
            marker.ns = 'apriltag'
            marker.id = data.apriltags[i].id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = data.posearray.poses[i].position.x
            marker.pose.position.y = data.posearray.poses[i].position.y
            marker.pose.position.z = data.posearray.poses[i].position.z
            marker.pose.orientation.x = data.posearray.poses[i].orientation.x
            marker.pose.orientation.y = data.posearray.poses[i].orientation.y
            marker.pose.orientation.z = data.posearray.poses[i].orientation.z
            marker.pose.orientation.w = data.posearray.poses[i].orientation.w
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.tag_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    tag_pub = TagPosePub()
    tag_pub.run()