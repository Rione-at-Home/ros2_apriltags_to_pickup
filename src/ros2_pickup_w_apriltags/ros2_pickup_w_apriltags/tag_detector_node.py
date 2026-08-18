#!/usr/bin/env python3
"""
tag_detector_node.py

Detects ArUco tags representing trash categories and publishes their
position (for approach/centering) and category ID (for sorting).

Handoff notes for the hardware team:
  - Topics published:
        /tag_pose  (geometry_msgs/PoseStamped) - position of the closest
                    valid tag, in the camera's frame. x = horizontal
                    offset (meters, negative=left/positive=right),
                    z = distance to the tag (meters).
        /tag_id    (std_msgs/Int32) - category ID of that same tag.
                    See CATEGORY_NAMES below for the ID -> category map.
  - Both messages are published together, in the same callback, so a
    /tag_pose message and the /tag_id message right after it always
    describe the same detection.
  - If nothing valid is in view, neither topic is published that frame
    (don't assume a fixed rate - use a timeout on your side, as we do
    in the state machine skeleton).
  - A live debug window ("Live Tag Detections") opens automatically
    showing what the detector sees. Requires a display (X11/local
    desktop) - if running headless over SSH, use `ssh -X`.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32


# Tag ID -> trash category. Extend/edit this if categories change -
# nothing else in this file needs to know the meaning of an ID.
CATEGORY_NAMES = {
    0: "Burnable (kanenengomi)",
    1: "PET Bottle (petbotoru)",
    2: "Can (kan)",
    3: "Non-burnable (funenengomi)",  # stretch-goal category
}


class TagDetectorNode(Node):

    def __init__(self):
        super().__init__("tag_detector_node")

        self.bridge = CvBridge()

        # --- ArUco setup ---
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # Tuned for small/low-res markers. If detection feels unreliable
        # on your camera/resolution, these are the first knobs to check -
        # see the README troubleshooting section.
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.polygonalApproxAccuracyRate = 0.05

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # --- Camera intrinsics ---
        # Deliberately NOT hardcoded to a specific resolution. These are
        # derived from the first incoming frame instead, so this node
        # keeps working correctly even if the camera or its resolution
        # changes later. This is still an approximation (focal length is
        # estimated from a fixed ratio, not measured) - if you need
        # precise distance readings, replace this with a proper
        # checkerboard calibration (cv2.calibrateCamera) and hardcode
        # the result instead.
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.MARKER_SIZE = 0.05  # meters - MUST match your printed marker exactly.
                                  # See generate_tags.py and the README for printing
                                  # instructions. A wrong value here biases every
                                  # distance reading by the same proportion.

        self.VALID_TAG_IDS = tuple(CATEGORY_NAMES.keys())

        # --- Subscribers ---
        # qos_profile_sensor_data matches the BEST_EFFORT QoS that camera
        # driver nodes (v4l2_camera, usb_cam) typically use. A QoS
        # mismatch here can silently drop most frames - if /tag_pose
        # looks abnormally slow or gappy, check this first.
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, qos_profile_sensor_data
        )

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, "/tag_pose", 10)
        self.id_pub = self.create_publisher(Int32, "/tag_id", 10)

        self.get_logger().info("Tag Detector Node initialized.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.camera_matrix is None:
            self._init_camera_matrix(frame.shape)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            flat_ids = ids.flatten()
            self.get_logger().info(
                f"Detected IDs: {flat_ids.tolist()}", throttle_duration_sec=0.5
            )

            # Ignore anything that isn't one of our known categories -
            # stray markers, QR codes, or other patterns the detector
            # mistakes for a tag will show up here as unexpected IDs.
            valid_indices = [
                i for i, tag_id in enumerate(flat_ids) if tag_id in self.VALID_TAG_IDS
            ]

            if valid_indices:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs
                )

                # If multiple valid tags are visible at once, target the
                # closest one.
                best_idx = min(valid_indices, key=lambda i: tvecs[i][0][2])
                tvec = tvecs[best_idx][0]
                rvec = rvecs[best_idx][0]
                tag_id = int(flat_ids[best_idx])
                category = CATEGORY_NAMES.get(tag_id, "UNKNOWN")

                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.MARKER_SIZE / 2.0
                )
                cv2.putText(
                    frame, f"TARGET: {category} (ID {tag_id}) | Z: {tvec[2]:.2f}m",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2
                )

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_frame"
                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])
                self.pose_pub.publish(pose_msg)

                id_msg = Int32()
                id_msg.data = tag_id
                self.id_pub.publish(id_msg)

        cv2.imshow("Live Tag Detections", frame)
        cv2.waitKey(1)  # required for OpenCV to actually paint the window

    def _init_camera_matrix(self, frame_shape):
        """Derive approximate intrinsics from the actual frame size on
        first callback, rather than assuming a fixed resolution."""
        height, width = frame_shape[0], frame_shape[1]
        focal_length = width * 0.9375  # empirically-matched ratio, see file docstring
        self.camera_matrix = np.array(
            [[focal_length, 0, width / 2.0],
             [0, focal_length, height / 2.0],
             [0, 0, 1]], dtype=np.float32
        )
        self.get_logger().info(
            f"Camera matrix initialized for {width}x{height} frame "
            f"(approximate - run a proper calibration for precise distances)."
        )


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    rclpy.spin(node)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()