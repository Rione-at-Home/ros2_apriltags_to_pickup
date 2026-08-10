#!/usr/bin/env python3
"""
gui.py

PyQt5 control panel for the TurtleBot2.5 competition robot
(TurtleBot2 Kobuki base + Crane+ 5-DOF arm).

This GUI does NOT contain any robot motion logic itself.
All motion is implemented in ArmController (arm.py) and
BaseController (base.py). The GUI only builds widgets and
calls methods on those two controller objects.

Because the controller methods use time.sleep() to wait
between movements, they are run on a background QThread
("RobotWorker") so the GUI never freezes while the robot
is moving.
"""

import sys
import math

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QSlider,
    QPushButton,
    QDoubleSpinBox,
    QSpinBox,
    QFrame,
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal

from .arm import ArmController
from .base import BaseController


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class RosNode(Node):
    """
    Thin ROS2 node that owns the ArmController and BaseController.
    All publishing happens inside those controllers, not here.
    """

    def __init__(self):
        super().__init__("crane_plus_gui")

        self.arm = ArmController(self)
        self.base = BaseController(self)


# ---------------------------------------------------------------------------
# Background worker
# ---------------------------------------------------------------------------
class RobotWorker(QThread):
    """
    Runs a single blocking robot action (e.g. arm.pick_can(),
    base.forward(0.5)) on a background thread so the Qt event
    loop keeps running and the GUI stays responsive.

    Usage:
        worker = RobotWorker(lambda: self.arm.pick_can())
        worker.finished_ok.connect(self.some_slot)
        worker.start()
    """

    finished_ok = pyqtSignal()
    failed = pyqtSignal(str)

    def __init__(self, action):
        super().__init__()
        self.action = action

    def run(self):
        try:
            self.action()
            self.finished_ok.emit()
        except Exception as exc:  # noqa: BLE001 - surface any error to the GUI
            self.failed.emit(str(exc))


# ---------------------------------------------------------------------------
# Main GUI
# ---------------------------------------------------------------------------
class CraneGUI(QWidget):
    """
    Main control panel window. Split into two clearly separated
    sections: Arm Controls and TurtleBot Base Controls.
    """

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.arm = ros_node.arm
        self.base = ros_node.base

        # Keep a reference to the active worker so it isn't
        # garbage-collected while running, and so we can disable
        # buttons while a movement is in progress.
        self.active_worker = None

        self.pick_state = "idle"

        self.joint_names = [
            "crane_plus_joint1",
            "crane_plus_joint2",
            "crane_plus_joint3",
            "crane_plus_joint4",
            "crane_plus_joint_hand",
        ]

        self.sliders = []
        self.value_labels = []

        self.setWindowTitle("TurtleBot2.5 Control Panel")
        self.resize(1000, 750)

        self._build_ui()

    # -------------------------------------------------------------
    # UI construction
    # -------------------------------------------------------------
    def _build_ui(self):
        """Top-level layout: title, status, arm section, base section."""

        main_layout = QVBoxLayout()

        title = QLabel("TurtleBot2.5 Robot Control Panel")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        main_layout.addWidget(title)

        self.status_label = QLabel("Status: Ready")
        self.status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status_label)

        main_layout.addWidget(self.create_arm_section())
        main_layout.addWidget(self.create_base_section())

        self.setLayout(main_layout)

        # Initialize speed label/publish using the slider's starting value.
        self.update_speed(self.speed_slider.value())

    def _make_separator(self):
        """Small helper to draw a horizontal divider line."""
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    # -------------------------------------------------------------
    # Arm section
    # -------------------------------------------------------------
    def create_arm_section(self):
        """
        Build the 'Arm Controls' group box: speed slider, joint
        sliders, and arm action buttons.
        """

        group = QGroupBox("Arm Controls")
        layout = QVBoxLayout()

        layout.addLayout(self.create_speed_section())
        layout.addWidget(self._make_separator())

        for joint_name in self.joint_names:
            layout.addLayout(self._create_joint_row(joint_name))

        layout.addWidget(self._make_separator())
        layout.addLayout(self.create_arm_buttons())

        group.setLayout(layout)
        return group

    def create_speed_section(self):
        """Build the speed slider row."""

        row = QHBoxLayout()

        speed_text = QLabel("Speed")
        speed_text.setMinimumWidth(180)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(25)

        self.speed_value_label = QLabel("25%")
        self.speed_value_label.setMinimumWidth(120)

        self.speed_slider.valueChanged.connect(self.update_speed)

        row.addWidget(speed_text)
        row.addWidget(self.speed_slider)
        row.addWidget(self.speed_value_label)

        return row

    def _create_joint_row(self, joint_name):
        """Build a single joint slider row (label + slider + value)."""

        row = QHBoxLayout()

        joint_label = QLabel(joint_name)
        joint_label.setMinimumWidth(180)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(-150)
        slider.setMaximum(150)
        slider.setValue(0)

        value_label = QLabel("0° (0.00 rad)")
        value_label.setMinimumWidth(150)

        slider.valueChanged.connect(
            lambda value, lbl=value_label: self.update_joint_label(lbl, value)
        )
        slider.valueChanged.connect(self.publish_slider_positions)

        self.sliders.append(slider)
        self.value_labels.append(value_label)

        row.addWidget(joint_label)
        row.addWidget(slider)
        row.addWidget(value_label)

        return row

    def create_arm_buttons(self):
        """Build the row of arm action buttons (Home, Pick, Place, etc.)."""

        row = QHBoxLayout()

        self.home_button = QPushButton("Home Position")
        self.home_button.clicked.connect(self.home_position)

        self.pick_button = QPushButton("Pick Can")
        self.pick_button.clicked.connect(self.pick_can)

        self.placeLeft_button = QPushButton("Place Can Left")
        self.placeLeft_button.clicked.connect(self.place_can_left)
        self.placeLeft_button.setEnabled(False)

        self.placeRight_button = QPushButton("Place Can Right")
        self.placeRight_button.clicked.connect(self.place_can_right)
        self.placeRight_button.setEnabled(False)

        self.catapult_button = QPushButton("Catapult Can")
        self.catapult_button.clicked.connect(self.catapult_can)

        row.addWidget(self.home_button)
        row.addWidget(self.pick_button)
        row.addWidget(self.placeLeft_button)
        row.addWidget(self.placeRight_button)
        row.addWidget(self.catapult_button)

        return row

    # -------------------------------------------------------------
    # Base section
    # -------------------------------------------------------------
    def create_base_section(self):
        """
        Build the 'TurtleBot Base Controls' group box: distance
        spin box + forward/backward, angle spin box + left/right,
        wait spin box, and stop.
        """

        group = QGroupBox("TurtleBot Base Controls")
        layout = QVBoxLayout()

        layout.addLayout(self._create_distance_row())
        layout.addLayout(self._create_angle_row())
        layout.addLayout(self._create_wait_row())
        layout.addLayout(self.create_buttons())

        group.setLayout(layout)
        return group

    def _create_distance_row(self):
        """Distance spin box plus Forward / Backward buttons."""

        row = QHBoxLayout()

        label = QLabel("Distance (m)")
        label.setMinimumWidth(180)

        self.distance_spin = QDoubleSpinBox()
        self.distance_spin.setRange(0.05, 5.00)
        self.distance_spin.setSingleStep(0.05)
        self.distance_spin.setValue(0.50)
        self.distance_spin.setDecimals(2)

        self.forward_button = QPushButton("Forward")
        self.forward_button.clicked.connect(self.drive_forward)

        self.backward_button = QPushButton("Backward")
        self.backward_button.clicked.connect(self.drive_backward)

        row.addWidget(label)
        row.addWidget(self.distance_spin)
        row.addWidget(self.forward_button)
        row.addWidget(self.backward_button)

        return row

    def _create_angle_row(self):
        """Angle spin box plus Turn Left / Turn Right buttons."""

        row = QHBoxLayout()

        label = QLabel("Angle (deg)")
        label.setMinimumWidth(180)

        self.angle_spin = QSpinBox()
        self.angle_spin.setRange(0, 360)
        self.angle_spin.setSingleStep(5)
        self.angle_spin.setValue(90)

        self.turn_left_button = QPushButton("Turn Left")
        self.turn_left_button.clicked.connect(self.turn_left)

        self.turn_right_button = QPushButton("Turn Right")
        self.turn_right_button.clicked.connect(self.turn_right)

        row.addWidget(label)
        row.addWidget(self.angle_spin)
        row.addWidget(self.turn_left_button)
        row.addWidget(self.turn_right_button)

        return row

    def _create_wait_row(self):
        """Wait-time spin box plus the Wait button."""

        row = QHBoxLayout()

        label = QLabel("Wait (s)")
        label.setMinimumWidth(180)

        self.wait_spin = QDoubleSpinBox()
        self.wait_spin.setRange(0.5, 10.0)
        self.wait_spin.setSingleStep(0.5)
        self.wait_spin.setValue(1.0)
        self.wait_spin.setDecimals(1)

        self.wait_button = QPushButton("Wait")
        self.wait_button.clicked.connect(self.base_wait)

        row.addWidget(label)
        row.addWidget(self.wait_spin)
        row.addWidget(self.wait_button)

        return row

    def create_buttons(self):
        """The Stop button row (kept separate per the requested layout)."""

        row = QHBoxLayout()

        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.base_stop)

        row.addWidget(self.stop_button)

        return row

    # -------------------------------------------------------------
    # Status helper
    # -------------------------------------------------------------
    def update_status(self, text):
        """Update the status label shown at the top of the window."""
        self.status_label.setText(f"Status: {text}")

    # -------------------------------------------------------------
    # Background task runner
    # -------------------------------------------------------------
    def run_in_background(self, action, busy_text, done_text=None,
                           buttons_to_disable=None):
        """
        Run `action` (a no-arg callable) on a RobotWorker thread so the
        GUI does not freeze while the controller sleeps between moves.

        buttons_to_disable: optional list of QPushButtons to disable
        while the action runs, re-enabled automatically when it finishes.
        """

        if self.active_worker is not None:
            # A previous action is still running; ignore the new request.
            self.update_status("Busy - please wait for current action to finish")
            return

        buttons_to_disable = buttons_to_disable or []

        self.update_status(busy_text)
        for button in buttons_to_disable:
            button.setEnabled(False)

        worker = RobotWorker(action)

        def on_done():
            if done_text:
                self.update_status(done_text)
            for button in buttons_to_disable:
                button.setEnabled(True)
            self.active_worker = None

        def on_error(message):
            self.update_status(f"Error: {message}")
            for button in buttons_to_disable:
                button.setEnabled(True)
            self.active_worker = None

        worker.finished_ok.connect(on_done)
        worker.failed.connect(on_error)

        self.active_worker = worker
        worker.start()

    # -------------------------------------------------------------
    # Speed / joint slider handlers
    # -------------------------------------------------------------
    def update_speed(self, value):
        """Slider moved -> update the % label and tell the arm controller."""
        self.speed_value_label.setText(f"{value}%")
        self.arm.set_speed(value)

    def update_joint_label(self, label, deg):
        """Update a joint's text label to show degrees and radians."""
        rad = math.radians(deg)
        label.setText(f"{deg:>4d}° ({rad:+.2f} rad)")

    def publish_slider_positions(self):
        """
        Manual jogging: read all slider values and send them straight
        to the arm via ArmController.move() (no extra motion logic here).
        """
        pose_deg = [slider.value() for slider in self.sliders]
        self.arm.move(pose_deg)

    def _sync_sliders_to_pose(self, pose_deg):
        """
        After a scripted arm action, move the sliders to match the
        arm's new position without re-publishing (avoids feedback loops).
        """
        for slider, angle in zip(self.sliders, pose_deg):
            slider.blockSignals(True)
            slider.setValue(int(angle))
            slider.blockSignals(False)
            label = self.value_labels[self.sliders.index(slider)]
            self.update_joint_label(label, int(angle))

    # -------------------------------------------------------------
    # Arm actions - just call ArmController methods
    # -------------------------------------------------------------
    def home_position(self):
        self.pick_state = "idle"
        self.placeLeft_button.setEnabled(False)
        self.placeRight_button.setEnabled(False)

        self.run_in_background(
            self.arm.home,
            busy_text="Returning Home",
            done_text="Home position reached",
            buttons_to_disable=[
                self.home_button, self.pick_button,
                self.placeLeft_button, self.placeRight_button,
                self.catapult_button,
            ],
        )

    def pick_can(self):
        self.pick_state = "picking"

        def action():
            self.arm.pick_can()
            self.arm.lift()

        def done():
            self.pick_state = "grasped"
            self.placeLeft_button.setEnabled(True)
            self.placeRight_button.setEnabled(True)

        self.run_in_background(
            action,
            busy_text="Picking up can...",
            done_text="Can lifted. Choose Place Left/Right or Catapult.",
            buttons_to_disable=[
                self.home_button, self.pick_button, self.catapult_button,
            ],
        )
        # Re-enable place buttons once the action completes successfully.
        if self.active_worker is not None:
            self.active_worker.finished_ok.connect(done)

    def place_can_left(self):
        if self.pick_state != "grasped":
            return

        def done():
            self.pick_state = "idle"
            self.placeLeft_button.setEnabled(False)
            self.placeRight_button.setEnabled(False)

        self.run_in_background(
            self.arm.place_left,
            busy_text="Placing can on the left...",
            done_text="Can placed successfully.",
            buttons_to_disable=[
                self.home_button, self.pick_button,
                self.placeLeft_button, self.placeRight_button,
                self.catapult_button,
            ],
        )
        if self.active_worker is not None:
            self.active_worker.finished_ok.connect(done)

    def place_can_right(self):
        if self.pick_state != "grasped":
            return

        def done():
            self.pick_state = "idle"
            self.placeLeft_button.setEnabled(False)
            self.placeRight_button.setEnabled(False)

        self.run_in_background(
            self.arm.place_right,
            busy_text="Placing can on the right...",
            done_text="Can placed successfully.",
            buttons_to_disable=[
                self.home_button, self.pick_button,
                self.placeLeft_button, self.placeRight_button,
                self.catapult_button,
            ],
        )
        if self.active_worker is not None:
            self.active_worker.finished_ok.connect(done)

    def catapult_can(self):
        def done():
            self.pick_state = "idle"
            self.placeLeft_button.setEnabled(False)
            self.placeRight_button.setEnabled(False)

        self.run_in_background(
            self.arm.catapult,
            busy_text="Catapulting can...",
            done_text="Can thrown.",
            buttons_to_disable=[
                self.home_button, self.pick_button,
                self.placeLeft_button, self.placeRight_button,
                self.catapult_button,
            ],
        )
        if self.active_worker is not None:
            self.active_worker.finished_ok.connect(done)

    # -------------------------------------------------------------
    # Base actions - just call BaseController methods
    # -------------------------------------------------------------
    def drive_forward(self):
        distance = self.distance_spin.value()
        self.run_in_background(
            lambda: self.base.forward(distance),
            busy_text=f"Driving forward {distance:.2f} m",
            done_text="Forward move complete",
            buttons_to_disable=[
                self.forward_button, self.backward_button,
                self.turn_left_button, self.turn_right_button,
            ],
        )

    def drive_backward(self):
        distance = self.distance_spin.value()
        self.run_in_background(
            lambda: self.base.backward(distance),
            busy_text=f"Driving backward {distance:.2f} m",
            done_text="Backward move complete",
            buttons_to_disable=[
                self.forward_button, self.backward_button,
                self.turn_left_button, self.turn_right_button,
            ],
        )

    def turn_left(self):
        angle = self.angle_spin.value()
        self.run_in_background(
            lambda: self.base.left(angle),
            busy_text=f"Turning left {angle}°",
            done_text="Turn complete",
            buttons_to_disable=[
                self.forward_button, self.backward_button,
                self.turn_left_button, self.turn_right_button,
            ],
        )

    def turn_right(self):
        angle = self.angle_spin.value()
        self.run_in_background(
            lambda: self.base.right(angle),
            busy_text=f"Turning right {angle}°",
            done_text="Turn complete",
            buttons_to_disable=[
                self.forward_button, self.backward_button,
                self.turn_left_button, self.turn_right_button,
            ],
        )

    def base_wait(self):
        seconds = self.wait_spin.value()
        self.run_in_background(
            lambda: self.base.wait(seconds),
            busy_text=f"Waiting {seconds:.1f} s",
            done_text="Wait complete",
            buttons_to_disable=[self.wait_button],
        )

    def base_stop(self):
        # Stop is instantaneous, no need to run on a background thread.
        self.base.stop()
        self.update_status("Base stopped")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():

    rclpy.init()

    ros_node = RosNode()

    app = QApplication(sys.argv)

    gui = CraneGUI(ros_node)
    gui.show()

    # Pump ROS2 callbacks alongside the Qt event loop.
    ros_timer = QTimer()
    ros_timer.timeout.connect(
        lambda: rclpy.spin_once(ros_node, timeout_sec=0.0)
    )
    ros_timer.start(10)

    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()