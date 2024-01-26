#!/usr/bin/env python3
# coding: utf-8
#
# Copyright(C) 2023 Isao Hara
#  All rights reserved
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
import os
import copy
import cv2
import yaml
import numpy as np
import math
import traceback

from cv_bridge import CvBridge
import tf_transformations
import tf2_ros
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import transformations

from cv_aruco_interfaces.srv import FindArucoMarker
#####
def get_predefined_dictionary(name):
  try:
    dict_value = cv2.aruco.__getattrute__(name)
  except:
    dict_value = cv2.aruco.DICT_4X4_50
  dictionary = cv2.aruco.getPredefinedDictionary(dict_value)
  return dictionary

#
#  Vision Module
class ArUroMarker(rclpy.node.Node):
  def __init__(self, name="aruco_marker"):
    super().__init__(name)
    self.data={}
    self.point=None
    self.points=[]
    self.corners={}

    self.xyz=None
    self.rpy=None
    self.mtx=None
    self.dist=None

    #
    #
    self.init_params()
    #
    #
    self.dictionary_id = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
    self.dictionary = get_predefined_dictionary(self.dictionary_id)
    self.marker_length = self.get_parameter("marker_size").get_parameter_value().double_value

    self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
    self.bridge = CvBridge()
    #
    #
    self.init_ros()

  #
  #
  def init_params(self):
    #self.declare_parameter("marker_size", 0.038)
    self.declare_parameter("marker_size", 0.045)
    self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50")
    return

  #
  #
  def init_ros(self):
    self.sub_info = self.create_subscription(CameraInfo, "camera_info",
                              self.callback_camera_info, qos_profile_sensor_data)
    self.sub_img = self.create_subscription(Image,"image_raw",
                              self.callback_image, qos_profile_sensor_data)

    self.pub_result_img = self.create_publisher(Image, "result_image", 10)
    self.pub_result = self.create_publisher(PoseStamped, "result_pose", 10)

    self.sev_find_marker = self.create_service(FindArucoMarker, "find_aruco_marker", self.find_aruco_marker_service)
    return

  #
  #
  def callback_camera_info(self, msg):
    self.cam_info=msg
    self.mtx= np.reshape(np.array(msg.k), (3, 3))
    self.dist = np.array(msg.d)
    return

  #
  #
  def callback_image(self, msg):
    try:
      self.img_msg=msg
      img, markers = self.detect_ar_marker(msg)
      if img is not None:
        res = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.result_img_msg = res
        self.pub_result_img.publish(res)
    except:
      traceback.print_exc()
    return

  #
  #
  def find_aruco_marker_service(self, request, response):
    try:
      if request.frame.data:
        img_msg_ = request.frame
      else:
        img_msg_ = self.img_msg
      img, markers = self.detect_ar_marker( img_msg_ )
      self.result_img_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")

      if request.marker_id in markers:
        response.marker_id = request.marker_id
        response.pose = markers[request.marker_id].pose
        response.result = self.result_img_msg
        self.broadcast_tf(img_msg_, response.marker_id, response.pose)
      else:
        response.marker_id = -1
    except:
      traceback.print_exc()      
      response.marker_id = -1

    return response

  #
  # AR Marker
  def detect_ar_marker(self, img_msg, m_id=None):
    try:
      self.data={}
      self.markers_pose={}

      frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
      corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.dictionary)

      if ids is not None:
        rvec, tvec, obj_p = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                                 self.mtx, self.dist)
        for i in range(len(ids)):
          self.data[ids[i][0]] = sum(corners[i][0])/4
          self.corners[ids[i][0]] = corners[i][0]
          pose_stamped = self.estimate_marker_pose(frame, tvec, rvec, i)
          pose_stamped.header.frame_id = "cv_aruco_%d" % ids[i][0]
          self.pub_result.publish(pose_stamped)
          self.markers_pose[ids[i][0]] = pose_stamped
          self.broadcast_tf(img_msg, ids[i][0], pose_stamped.pose)

        ar_image = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return ar_image, self.markers_pose
    except:
      traceback.print_exc()
    return None,None

  #
  #
  def estimate_marker_pose(self, frame, tvec, rvec, n):
    try:
      r_mtx = np.eye(4)
      r_mtx[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[n][0]))[0]
      pose = PoseStamped()
      pose.pose.position.x = tvec[n][0][0]
      pose.pose.position.y = tvec[n][0][1]
      pose.pose.position.z = tvec[n][0][2]
      quat = tf_transformations.quaternion_from_matrix(r_mtx)
      pose.pose.orientation.x = quat[0]
      pose.pose.orientation.y = quat[1]
      pose.pose.orientation.z = quat[2]
      pose.pose.orientation.w = quat[3]

      cv2.drawFrameAxes(frame, self.mtx, self.dist, rvec[n], tvec[n], self.marker_length/2)
      return pose
    except:
      #traceback.print_exc()
      return None

  #
  #
  def broadcast_tf(self, img_msg, marker_id, pose):
    trans_ = TransformStamped()
    trans_.header = copy.deepcopy(img_msg.header)
    trans_.child_frame_id = "cv_aruco_%d" % marker_id
    trans_.transform.translation.x = pose.position.x
    trans_.transform.translation.y = pose.position.y
    trans_.transform.translation.z = pose.position.z
    trans_.transform.rotation.x = pose.orientation.x
    trans_.transform.rotation.y = pose.orientation.y
    trans_.transform.rotation.z = pose.orientation.z
    trans_.transform.rotation.w = pose.orientation.w
    self.tf_broadcaster.sendTransform(trans_)
    return

#####
#  M A I N
def main():
  rclpy.init()
  node = ArUroMarker()
  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()


#
#
if __name__ == '__main__':
    main()
