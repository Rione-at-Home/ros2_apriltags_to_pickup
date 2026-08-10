#!/usr/bin/env python3

#
## challenge_node.py
#
#  This program defines the main node for the challenge. It initializes the 
#  robot and provides a template for implementing the challenge solution.
#
#
import enum
import rclpy
from rclpy.node import Node
import time

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


        # Camera Feedback fields
        self.target_visible = False
        self.target_x_offset = 0.0
        self.target_z_dist = 0.0
        self.target_id = -1


        # Control gains and thresholds
        self.PICKUP_DISTANCE = 0.35
        self.KP_ANGULAR = 0.8

    

       




def main(args=None):

    rclpy.init(args=args)

    node = ChallengeNode()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()