#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

class AurcoMarkerNode(Node):

  def __init__(self):
    super().__init__('aruco_node')
    self.subscription = self.create_subscription(
      TransformStamped, 
      "/detected/marker", 
      self.marker_callback, 
      1
    )
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.target_marker = {}


  def marker_callback(self, msg : TransformStamped):
    print("marker_callback")

    if msg.child_frame_id not in self.target_marker.keys():
      self.target_marker[msg.child_frame_id] = time.time()

    detected_time = self.target_marker[msg.child_frame_id]
    elapsed_time = time.time() - detected_time

    if elapsed_time <= 2 :
      return


    print(self.target_marker)
    
    
    try:
      transform = self.tf_buffer.lookup_transform(
        target_frame='link1',
        source_frame=msg.child_frame_id,
        time=rclpy.time.Time()
      )
    except Exception as e:
      self.get_logger().error(f"depth_callback Exception : {e}")
  

def main(args=None):
  rclpy.init(args=args)
  aruco_node = AurcoMarkerNode()
  rclpy.spin(aruco_node)
  aruco_node.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()  
