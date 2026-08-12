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

        # Tuned for small/low-res markers at 320x240
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerPerimeterRate = 0.02 
        self.parameters.polygonalApproxAccuracyRate = 0.05

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Camera intrinsics - scaled for 640x480 resolution
        self.camera_matrix = np.array(
            [[600, 0, 320],
             [0, 600, 240],
             [0, 0, 1]], dtype=np.float32
        )
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.MARKER_SIZE = 0.05  # 50mm

        self.VALID_TAG_IDS = (0, 1, 17)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, qos_profile_sensor_data
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/tag_pose", 10)
        self.id_pub = self.create_publisher(Int32, "/tag_id", 10)

        self.get_logger().info("Tag Detector Node initialized with OpenCV Visualizer.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(gray)

        # If we found at least one marker
        if ids is not None and len(ids) > 0:
            # Draw outlines around ALL detected markers for visual context
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            flat_ids = ids.flatten()
            self.get_logger().info(
                f"Detected IDs: {flat_ids.tolist()}", throttle_duration_sec=0.5
            )

            # Filter out anything that isn't one of our expected tags.
            valid_indices = [
                i for i, tag_id in enumerate(flat_ids) if tag_id in self.VALID_TAG_IDS
            ]

            # If there are valid targets in the frame
            if valid_indices:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs
                )

                # If multiple valid tags are visible, pick the closest one (smallest z).
                best_idx = min(valid_indices, key=lambda i: tvecs[i][0][2])
                tvec = tvecs[best_idx][0]
                rvec = rvecs[best_idx][0]
                tag_id = int(flat_ids[best_idx])
                
                # Draw 3D axes on the chosen target (length is half the marker size)
                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs, 
                    rvec, tvec, self.MARKER_SIZE / 2.0
                )
                
                # Add text overlay showing the targeted ID and distance
                cv2.putText(
                    frame, f"TARGET: ID {tag_id} | Z: {tvec[2]:.2f}m", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.6, (0, 255, 0), 2
                )

                # Publish Pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_frame"
                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])
                self.pose_pub.publish(pose_msg)

                # Publish ID
                id_msg = Int32()
                id_msg.data = tag_id
                self.id_pub.publish(id_msg)

        # Show the annotated frame using OpenCV
        cv2.imshow("Live Tag Detections", frame)
        cv2.waitKey(1)  # Required to process GUI events and actually render the window


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    rclpy.spin(node)
    
    # Clean up OpenCV windows on shutdown
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()