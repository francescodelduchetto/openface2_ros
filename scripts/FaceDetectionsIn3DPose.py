#!/usr/bin/env python
import cv2
import rospy
import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
from openface2_ros.msg import Faces

class FaceDetectionsIn3DPose():

    def __init__(self, faces_topic="/openface2/faces", poses_topic="/openface2/bounding_box_centres", confidence_thr=0.4):
        self.confidence_thr = confidence_thr

        rospy.Subscriber(faces_topic, Faces, self.faces_cb)

        self.poseArray_pub = rospy.Publisher(poses_topic, PoseArray, queue_size=1)

        print "initialized"

    def faces_cb(self, msg):

        # cropped_imgs = []
        # centers_2d = []
        # median_distances = []
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "interaction_rs_color_optical_frame"
        for face in msg.faces:
            if face.confidence < self.confidence_thr:
                continue

            pose = Pose()
            pose.position = face.head_pose.position
            pose.orientation = face.head_pose.orientation

            pose_array.poses.append(pose)

        self.publish_poses(pose_array)


if __name__ == '__main__':
    rospy.init_node("openface2_3D_pose_node")

    FaceDetectionsIn3DPose()

    rospy.spin()
