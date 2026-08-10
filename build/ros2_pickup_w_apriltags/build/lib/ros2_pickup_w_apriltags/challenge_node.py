#!/usr/bin/env python3
import enum
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from .robot import Robot


class State(enum.Enum):
    SEARCHING = 1
    APPROACHING = 2
    PICKING = 3
    SORTING = 4
    FINISHED = 5


class ChallengeNode(Node):

    def __init__(self):
        super().__init__("challenge_node")

        self.robot = Robot(self)

        self.state = State.SEARCHING

        # Camera feedback data
        self.target_visible = False
        self.last_tag_time = self.get_clock().now()
        self.target_x_offset = 0.0  # Horizontal alignment error (meters)
        self.target_z_dist = 0.0    # Distance to tag (meters)
        self.target_id = -1          # Tag ID (0 = Left/Recycling, 1 = Right/Trash)

        # Control gains & thresholds
        self.PICKUP_DISTANCE = 0.35  # Stopping distance for Crane+ arm reach
        self.KP_ANGULAR = 0.8        # Steering P-gain
        self.KP_LINEAR = 0.4         # Drive P-gain

        # Subscriptions for Tag Detection Node
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/tag_pose",
            self.tag_pose_callback,
            10
        )
        self.id_sub = self.create_subscription(
            Int32,
            "/tag_id",
            self.tag_id_callback,
            10
        )

        self.get_logger().info("Challenge Node initialized! Starting control loop...")

        # Run control loop at 10 Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.control_loop)

    def tag_pose_callback(self, msg: PoseStamped):
        """Updates tag position published by the vision node."""
        self.target_visible = True
        self.last_tag_time = self.get_clock().now()
        self.target_x_offset = msg.pose.position.x
        self.target_z_dist = msg.pose.position.z

    def tag_id_callback(self, msg: Int32):
        """Updates tag ID published by the vision node."""
        self.target_id = msg.data

    def control_loop(self):
        """Main State Machine Loop."""

        # Timeout: Mark invisible if no tag frame received in over 0.5s
        time_since_seen = (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9
        if time_since_seen > 2:
            self.target_visible = False

        # State 1 - SEARCHING
        if self.state == State.SEARCHING:
            if not self.target_visible:
                self.get_logger().info("Searching for paper bag...", throttle_duration_sec=2)
                self.robot.base.drive(linear=0.0, angular=0.3)
            else:
                self.get_logger().info("Tag spotted! Switching to APPROACHING.")
                self.robot.base.stop()
                self.state = State.APPROACHING

        # STATE 2: APPROACHING 
        elif self.state == State.APPROACHING:
            if not self.target_visible:
                self.get_logger().warn("Lost sight of tag! Returning to SEARCHING.")
                self.state = State.SEARCHING
                return

            dist_error = self.target_z_dist - self.PICKUP_DISTANCE
            self.get_logger().info(
                f"z={self.target_z_dist:.3f} x_off={self.target_x_offset:.3f} dist_err={dist_error:.3f}",
                throttle_duration_sec=0.5
            )


            # Stop condition: close enough and well-centered
            if dist_error <= 0.02 and abs(self.target_x_offset) < 0.05:
                self.get_logger().info("Reached target! Stopping base.")
                self.robot.base.stop()
                self.state = State.PICKING
                return

            # Angular speed calculation
            angular_speed = -self.KP_ANGULAR * self.target_x_offset
            angular_speed = max(-0.4, min(0.4, angular_speed))

            # Only drive forward once x_offset is within ~8cm of center
            if abs(self.target_x_offset) > 0.08:
                linear_speed = 0.0
            else:
                linear_speed = self.KP_LINEAR * dist_error
                linear_speed = max(0.0, min(0.2, linear_speed))

            self.robot.base.drive(linear=linear_speed, angular=angular_speed)

        # State 3: PICKING 
        elif self.state == State.PICKING:
            self.get_logger().info("Picking up paper bag...")
            self.robot.arm.pick_can()
            self.robot.arm.lift()
            self.state = State.SORTING

        # State 4: SORTING
        elif self.state == State.SORTING:
            self.get_logger().info(f"Sorting bag with Tag ID: {self.target_id}")

            if self.target_id == 0:
                self.get_logger().info("Tag 0: Placing Left")
                self.robot.arm.place_left()
            elif self.target_id == 1:
                self.get_logger().info("Tag 1: Placing Right")
                self.robot.arm.place_right()
            else:
                self.get_logger().warn("Unknown Tag ID! Defaulting to Left.")
                self.robot.arm.place_left()

            self.robot.arm.home()
            self.state = State.FINISHED

        # Stage 5: Finished
        elif self.state == State.FINISHED:
            self.robot.base.stop()
            self.get_logger().info("Mission complete!", throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    node = ChallengeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()