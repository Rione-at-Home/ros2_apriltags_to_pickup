#!/usr/bin/env python3

#
## base.py
#
#
#  This program defines the high-level interface for controlling the TurtleBot2 base.
#
#

import math
import time

from geometry_msgs.msg import Twist


class BaseController:

    def __init__(self, node):

        self.node = node

        self.cmd_pub = node.create_publisher(
            Twist,
            "/commands/velocity",
            10
        )

        #
        # Default speeds
        #
        self.linear_speed = 0.5      # m/s
        self.angular_speed = 0.70     # rad/s

    ###################################################
    # Low-level movement
    ###################################################

    def stop(self):
        for _ in range(10):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def drive(self, linear=0.0, angular=0.0):

        msg = Twist()

        msg.linear.x = linear
        msg.angular.z = angular

        self.cmd_pub.publish(msg)

    def driveTime(self, seconds, linear=0.0,angular=0.0):
        start_time = time.time()
        while time.time() - start_time < seconds:
            self.drive(linear, angular)
            time.sleep(0.1)
        self.stop()


    # Marvel was here.

    def forward(self, distance):

        duration = (distance / self.linear_speed)

        self.driveTime(seconds=duration, linear=self.linear_speed)

    def backward(self, distance):

        duration = (distance / self.linear_speed)

        self.driveTime(seconds=duration,linear=-self.linear_speed)

    def left(self, angle_deg):

        angle_rad = math.radians(angle_deg)

        duration = angle_rad / self.angular_speed

        self.drive(angular=self.angular_speed)

        time.sleep(duration)

        self.stop()

    def right(self, angle_deg):

        angle_rad = math.radians(angle_deg)

        duration = angle_rad / self.angular_speed

        self.drive(angular=-self.angular_speed)

        time.sleep(duration)

        self.stop()

    def wait(self, seconds):

        self.stop()

        time.sleep(seconds)