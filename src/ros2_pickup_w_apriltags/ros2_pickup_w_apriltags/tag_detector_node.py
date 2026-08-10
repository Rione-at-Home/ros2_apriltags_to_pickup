#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

        # Tuned for small/low-res markers at 320x240 - the defaults were
        # rejecting valid detections and causing gaps of 1-2+ seconds.
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerPerimeterRate = 0.02  # default 0.03, too strict at this res
        self.parameters.polygonalApproxAccuracyRate = 0.05

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Camera intrinsics - corrected for the actual 320x240 stream.
        # cx/cy must be the true image center (width/2, height/2), and
        # focal length scales down with resolution. These are still
        # approximate; run a proper checkerboard calibration when you
        # have time for accurate distance/offset readings.
        self.camera_matrix = np.array(
            [[300, 0, 160],
             [0, 300, 120],
             [0, 0, 1]], dtype=np.float32
        )
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.MARKER_SIZE = 0.05  # 50mm = 0.05 meters - verify against your printed marker

        # Only these tag IDs are valid targets. Anything else detected
        # in-frame (false positives, unrelated markers) is ignored.
        self.VALID_TAG_IDS = (0, 1)

        # Match QoS with the camera driver (v4l2_camera / usb_cam publish
        # sensor data as BEST_EFFORT by default).
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, qos_profile_sensor_data
        )

        self.pose_pub = self.create_publisher(PoseStamped, "/tag_pose", 10)
        self.id_pub = self.create_publisher(Int32, "/tag_id", 10)

        self.get_logger().info("Tag Detector Node initialized.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return

        flat_ids = ids.flatten()

        self.get_logger().info(
            f"Detected IDs: {flat_ids.tolist()}", throttle_duration_sec=0.5
        )

        # Filter out anything that isn't one of our expected tags.
        valid_indices = [
            i for i, tag_id in enumerate(flat_ids) if tag_id in self.VALID_TAG_IDS
        ]

        if not valid_indices:
            return  # nothing relevant in this frame, don't publish stale/garbage data

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs
        )

        # If multiple valid tags are visible, pick the closest one (smallest z).
        best_idx = min(valid_indices, key=lambda i: tvecs[i][0][2])
        tvec = tvecs[best_idx][0]
        tag_id = int(flat_ids[best_idx])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_frame"
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        id_msg = Int32()
        id_msg.data = tag_id

        self.pose_pub.publish(pose_msg)
        self.id_pub.publish(id_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
