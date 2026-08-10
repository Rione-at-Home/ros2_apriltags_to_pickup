#!/usr/bin/env python3

#
## challenge_node.py
#
#  This program defines the main node for the challenge. It initializes the 
#  robot and provides a template for implementing the challenge solution.
#
#

import rclpy
from rclpy.node import Node
import time

from .robot import Robot


class ChallengeNode(Node):

    def __init__(self):
        super().__init__("challenge_node")

        self.robot = Robot(self)

        self.get_logger().info("Starting challenge!")

        self.run()

    def run(self):
        """
        Write your challenge solution here.

        Available commands:

        Base:
            self.robot.base.forward(distance_m)
            self.robot.base.backward(distance_m)
            self.robot.base.left(angle_deg)
            self.robot.base.right(angle_deg)
            self.robot.base.wait(seconds)

        Arm:
            self.robot.arm.home()
            self.robot.arm.pick_can()
            self.robot.arm.lift()
            self.robot.arm.place_left()
            self.robot.arm.place_right()
            self.robot.arm.catapult()
        """
        
        
        #
        # Example program
        #

        #self.get_logger().info("Running example!")

        self.robot.arm.custom1()
        self.robot.base.forward(0.70)
        self.robot.arm.customgrab()
        self.robot.base.forward(0.40)
        for i in range(1, 57):
            self.robot.base.right(10)
        self.robot.base.forward(0.20)
        self.robot.arm.custom1()

       




def main(args=None):

    rclpy.init(args=args)

    node = ChallengeNode()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()