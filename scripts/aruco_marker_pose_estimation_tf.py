#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
 
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
 
class ArucoNode(Node):

  def __init__(self):
    super().__init__('aruco_node')

    self.declare_parameter("aruco_dictionary_name", "DICT_4X4_1000")
    self.declare_parameter("aruco_marker_side_length", 0.01)
     
    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
         
    self.mtx = None
    self.dst = None
    self.tfbroadcaster = TransformBroadcaster(self)
    self.bridge = CvBridge()

    self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
    self.aruco_parameters = cv2.aruco.DetectorParameters()
      
    self.subscription = self.create_subscription(
      Image, 
      "/camera/camera/color/image_raw", 
      self.listener_callback, 
      10
    )
    self.info_sub = self.create_subscription(
      CameraInfo, 
      "/camera/camera/depth/camera_info", 
      self.info_callback, 
      10
    )


  def info_callback(self, info_msg):
    self.info_msg = info_msg
    self.mtx = np.reshape(np.array(self.info_msg.k), (3, 3))
    self.dst = np.array(self.info_msg.d)

    self.destroy_subscription(self.info_sub)

  def listener_callback(self, data):  
    current_frame = self.bridge.imgmsg_to_cv2(data)
    
    corners, marker_ids, _ = cv2.aruco.detectMarkers(
      current_frame, self.aruco_dictionary, parameters=self.aruco_parameters
    )

    if marker_ids is None:
      return
    
    blue_BGR = (255, 0, 0)

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
      corners,
      self.aruco_marker_side_length,
      self.mtx,
      self.dst
    )

    for i, marker_id in enumerate(marker_ids):  

      (topLeft, topRight, bottomRight, bottomLeft) = corners[i][0]
      
      topRightPoint    = (int(topRight[0]),      int(topRight[1]))
      topLeftPoint     = (int(topLeft[0]),       int(topLeft[1]))
      bottomRightPoint = (int(bottomRight[0]),   int(bottomRight[1]))
      bottomLeftPoint  = (int(bottomLeft[0]),    int(bottomLeft[1]))

      cv2.circle(current_frame, topLeftPoint, 4, blue_BGR, -1)
      cv2.circle(current_frame, topRightPoint, 4, blue_BGR, -1)
      cv2.circle(current_frame, bottomRightPoint, 4, blue_BGR, -1)
      cv2.circle(current_frame, bottomLeftPoint, 4, blue_BGR, -1)


      t = TransformStamped()
      t.header.stamp = self.get_clock().now().to_msg()
      t.header.frame_id = 'camera_link'
      t.child_frame_id = f'aruco_marker_{marker_id}'
      
      # Store the translation (i.e. position) information
      t.transform.translation.x = tvecs[i][0][0]
      t.transform.translation.y = tvecs[i][0][1]
      t.transform.translation.z = tvecs[i][0][2]

      # Store the rotation information
      rotation_matrix = np.eye(4)
      rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
      r = R.from_matrix(rotation_matrix[0:3, 0:3])
      quat = r.as_quat()   
        
      # Quaternion format     
      t.transform.rotation.x = quat[0] 
      t.transform.rotation.y = quat[1] 
      t.transform.rotation.z = quat[2] 
      t.transform.rotation.w = quat[3] 

      self.tfbroadcaster.sendTransform(t)

    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)
   
def main(args=None):
  rclpy.init(args=args)
  aruco_node = ArucoNode()
  rclpy.spin(aruco_node)
  aruco_node.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()