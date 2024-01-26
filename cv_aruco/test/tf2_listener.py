#
#
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#
#
class Tf2FrameListener(Node):
  def __init__(self):
    super().__init__('tf2_frame_listener')

    # Declare and acquire `target_frame` parameter
    self.target_frame = self.declare_parameter(
        'target_frame', 'cv_aruco_0').get_parameter_value().string_value
    self.source_frame = self.declare_parameter(
        'source_frame', 'default_cam').get_parameter_value().string_value
    #
    # create Tf2 listener
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # Call on_timer function every second
    self.timer = self.create_timer(1.0, self.on_timer)

  def update_params(self):
    self.target_frame=self.get_parameter('target_frame').get_parameter_value().string_value
    self.source_frame=self.get_parameter('source_frame').get_parameter_value().string_value
    return
  #
  #
  def on_timer(self):
    self.update_params()
    res = self.get_transform(self.target_frame,self.source_frame)
    if res:
      print(self.target_frame, " <- ", self.source_frame)
      print(res.transform)
    return

  #
  #
  def get_transform(self, target, source):
    try:
      t_stamped = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
      return t_stamped
    except TransformException as ex:
      self.get_logger().info(
         f'Could not transform {target} to {source}: {ex}')
    return None

#
#
def main():
  rclpy.init()
  node = Tf2FrameListener()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  rclpy.shutdown()

#
#
if __name__ == '__main__':
  main()
