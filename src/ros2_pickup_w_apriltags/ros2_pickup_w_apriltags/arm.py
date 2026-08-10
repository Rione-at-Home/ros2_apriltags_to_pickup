#
## arm.py
#
#  This program defines the high-level interface for controlling the Crane+ arm. 
#  It provides methods for moving the arm to predefined poses, as well as for 
#  executing specific tasks such as picking up a can or placing it in a 
#  designated location.
#
#

#!/usr/bin/env python3

import math
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from . import poses


class ArmController:

    def __init__(self, node):

        self.node = node

        self.joint_pub = node.create_publisher(
            JointState,
            "/crane_plus_command",
            5
        )

        self.speed_pub = node.create_publisher(
            Int32,
            "/crane_plus_speed",
            5
        )

        self.joint_names = [
            "crane_plus_joint1",
            "crane_plus_joint2",
            "crane_plus_joint3",
            "crane_plus_joint4",
            "crane_plus_joint_hand",
        ]

    def set_speed(self, percent):

        msg = Int32()
        msg.data = percent

        self.speed_pub.publish(msg)

    def move(self, pose_deg):

        msg = JointState()

        msg.name = self.joint_names

        msg.position = [
            math.radians(angle)
            for angle in pose_deg
        ]

        self.joint_pub.publish(msg)

    def move_and_wait(self, pose_deg, wait_time=2.0):

        self.move(pose_deg)
        time.sleep(wait_time)

    def home(self):

        self.move_and_wait(poses.HOME)

    def pick_can(self):

        # self.move_and_wait(poses.PRE_PICK)
        # self.move_and_wait(poses.APPROACH)
        self.move_and_wait(poses.GRAB)

    def lift(self):

        self.move_and_wait(poses.LIFT)

    def place_left(self):

        self.move_and_wait(poses.LEFT_PLACE)
        self.move_and_wait(poses.LEFT_RELEASE)

    def place_right(self):

        self.move_and_wait(poses.RIGHT_PLACE)
        self.move_and_wait(poses.RIGHT_RELEASE)

    def catapult(self):

        self.move_and_wait(poses.THROW_READY)
        self.move_and_wait(poses.THROW, 0.15)
        self.move_and_wait(poses.THROW_RELEASE, 0.5)

    def close(self):

        self.move_and_wait(poses.GRAB2)
    
    def custom1(self):
        self.move_and_wait(poses.CUSTOM1)

    def customgrab(self):
        self.move_and_wait(poses.CUSTOMGRAB)

    