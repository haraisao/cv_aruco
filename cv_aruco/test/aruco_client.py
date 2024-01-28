import sys
import time
from cv_aruco_interfaces.srv import FindArucoMarker

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

#
#
class FindArucoMarkerClient(Node):
  def __init__(self):
    super().__init__('find_aruco_marker_client')
    self.marker_id = self.declare_parameter('marker_id', 1).get_parameter_value().integer_value
    self.sub_img = self.create_subscription(Image, 'image_raw', self.callback_image, 10)
    self.sub_info = self.create_subscription(CameraInfo, "camera_info", self.callback_camera_info, 10)

    #
    #
    self.client = self.create_client(FindArucoMarker, 'find_aruco_marker')
    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info("Service not available")

    self.req = FindArucoMarker.Request()
    self.img_msg=None
    self.cam_info=None
    #
    #
    self.timer_cb = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    self.timer = self.create_timer(1.0, self.on_timer, callback_group=self.timer_cb)

  #
  #
  def callback_image(self, msg):
    self.img_msg=msg
    return

  def callback_camera_info(self, msg):
    self.cam_info=msg
    return

  #
  #
  def request(self, marker_id):
    if self.img_msg is None:
      self.get_logger().info("No image")
      return
    self.req.marker_id = marker_id
    self.req.image = self.img_msg
    self.req.camera_info = self.cam_info
    self.future = self.client.call_async(self.req)
    #rclpy.spin_until_future_complete(self, self.future)
    while not self.future.done():
      time.sleep(0.5)
    return self.future.result()

  #
  #
  def update_params(self):
    self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
    return
  #
  #
  def on_timer(self):
    self.update_params()
    response = self.request(self.marker_id)

    if response:
      print(response.marker_id)
      print(response.pose)
    else:
      print("Fail to call", self.marker_id)
    return

#
#
def main():
    rclpy.init()

    node = FindArucoMarkerClient()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
      executor.spin()
    except:
      import traceback
      traceback.print_exc()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
