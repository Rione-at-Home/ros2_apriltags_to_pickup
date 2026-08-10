#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

class TagDetectorNode(Node):
    def __init__(self):
        super().__init__("tag_detector_node")
        
        self.bridge = CvBridge()
        
        # ArUco dictionary setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Camera intrinsics (approximate defaults for standard webcam)
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.MARKER_SIZE = 0.05  # 50mm = 0.05 meters

        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.pose_pub = self.create_publisher(PoseStamped, "/tag_pose", 10)
        self.id_pub = self.create_publisher(Int32, "/tag_id", 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # Estimate pose of the first detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs
            )
            
            tvec = tvecs[0][0]  # [x, y, z] relative to camera frame

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_frame"
            pose_msg.pose.position.x = float(tvec[0])
            pose_msg.pose.position.y = float(tvec[1])
            pose_msg.pose.position.z = float(tvec[2])

            id_msg = Int32()
            id_msg.data = int(ids[0][0])

            self.pose_pub.publish(pose_msg)
            self.id_pub.publish(id_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
