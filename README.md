# ros2_apriltags_to_pickup

## ROS 2 Challenge Node – ArUco Detection & Robot Sorting

This project implements a ROS 2 robot challenge where a robot uses a camera to detect an **ArUco marker**, approaches the detected target, picks it up, and sorts it to the left or right depending on the marker ID.

The system is split into two ROS 2 nodes:

1. **`TagDetectorNode`** – Processes camera images and detects ArUco markers.
2. **`ChallengeNode`** – Controls the robot using a state machine and the marker information.

---

## Overview

The robot follows this sequence:

```text
Camera
   │
   ▼
TagDetectorNode
   │
   ├── /tag_pose ──► Target position
   │
   └── /tag_id ───► Target ID
                       │
                       ▼
                ChallengeNode
                       │
          ┌────────────┴────────────┐
          │                         │
          ▼                         ▼
      Robot Base                Robot Arm
          │                         │
      Approach                 Pick & Sort
          │                         │
          └────────────┬────────────┘
                       ▼
                    FINISHED
```

The robot initially rotates until an ArUco marker is detected. Once detected, it uses proportional control to align itself with the marker and drive toward it. When the robot reaches the required distance, the arm picks up the object and places it according to the marker ID.

---

# Nodes

## 1. `TagDetectorNode`

The `TagDetectorNode` is responsible for:

* Receiving camera images.
* Converting ROS images to OpenCV images.
* Detecting ArUco markers.
* Estimating the marker's position relative to the camera.
* Publishing the marker position.
* Publishing the marker ID.

### Input

| Topic        | Message Type            | Description  |
| ------------ | ----------------------- | ------------ |
| `/image_raw` | `sensor_msgs/msg/Image` | Camera image |

### Outputs

| Topic       | Message Type                    | Description                     |
| ----------- | ------------------------------- | ------------------------------- |
| `/tag_pose` | `geometry_msgs/msg/PoseStamped` | Position of the detected marker |
| `/tag_id`   | `std_msgs/msg/Int32`            | ID of the detected marker       |

---

## 2. `ChallengeNode`

The `ChallengeNode` controls the robot and implements the main challenge state machine.

It receives the marker position and ID from `TagDetectorNode` and controls:

* The robot base.
* The robot arm.
* Searching behavior.
* Target approach.
* Picking.
* Sorting.
* Mission completion.

---

# State Machine

The challenge is implemented using five states.

```text
             ┌─────────────┐
             │  SEARCHING  │
             └──────┬──────┘
                    │
              Tag detected
                    │
                    ▼
             ┌─────────────┐
             │ APPROACHING │
             └──────┬──────┘
                    │
            Within pickup distance
                    │
                    ▼
              ┌─────────┐
              │ PICKING │
              └────┬────┘
                   │
               Pick complete
                   │
                   ▼
              ┌─────────┐
              │ SORTING │
              └────┬────┘
                   │
             Placement complete
                   │
                   ▼
             ┌──────────┐
             │ FINISHED │
             └──────────┘
```

### `SEARCHING`

The robot rotates its base slowly until an ArUco marker is detected.

```python
self.robot.base.drive(linear=0.0, angular=0.3)
```

When a marker is detected, the robot stops and changes to `APPROACHING`.

---

### `APPROACHING`

The robot uses proportional control to:

1. Center itself relative to the marker.
2. Drive toward the marker.
3. Stop when it reaches the pickup distance.

The angular velocity is calculated using:

```python
angular_speed = -self.KP_ANGULAR * self.target_x_offset
```

The linear velocity is calculated using:

```python
dist_error = self.target_z_dist - self.PICKUP_DISTANCE
linear_speed = self.KP_LINEAR * dist_error
```

Velocity limits are applied for safety:

```python
linear_speed = max(0.0, min(0.2, linear_speed))
angular_speed = max(-0.4, min(0.4, angular_speed))
```

The default pickup distance is:

```python
PICKUP_DISTANCE = 0.35
```

If the distance error is less than or equal to `0.02 m`, the robot stops and begins picking.

---

### `PICKING`

The robot performs the arm pickup sequence:

```python
self.robot.arm.pick_can()
self.robot.arm.lift()
```

After picking up the object, the state changes to `SORTING`.

---

### `SORTING`

The marker ID determines where the object is placed.

| Marker ID | Destination    |
| --------: | -------------- |
|       `0` | Left           |
|       `1` | Right          |
|     Other | Left (default) |

For marker ID `0`:

```python
self.robot.arm.place_left()
```

For marker ID `1`:

```python
self.robot.arm.place_right()
```

After placement, the arm returns to its home position:

```python
self.robot.arm.home()
```

The state then changes to `FINISHED`.

---

### `FINISHED`

The robot stops its base and reports that the mission is complete.

```python
self.robot.base.stop()
```

---

# Parameters and Control Values

The main control parameters are defined in `ChallengeNode`:

| Parameter             |     Default | Description                                             |
| --------------------- | ----------: | ------------------------------------------------------- |
| `PICKUP_DISTANCE`     |    `0.35 m` | Distance at which the robot stops to pick up the object |
| `KP_ANGULAR`          |       `0.8` | Proportional gain for angular steering                  |
| `KP_LINEAR`           |       `0.4` | Proportional gain for forward movement                  |
| Maximum linear speed  |   `0.2 m/s` | Safety limit for forward motion                         |
| Maximum angular speed | `0.4 rad/s` | Safety limit for rotation                               |
| Control frequency     |     `10 Hz` | Main control loop frequency                             |

---

# ArUco Marker Detection

The vision node uses OpenCV's ArUco detector.

The configured dictionary is:

```python
cv2.aruco.DICT_4X4_50
```

The expected marker size is:

```python
MARKER_SIZE = 0.05
```

This corresponds to a marker size of:

```text
50 mm = 0.05 m
```

The detector estimates the marker position as:

```text
[x, y, z]
```

relative to the camera.

These values are published in a `PoseStamped` message.

---

# Camera Configuration

The current implementation uses approximate camera intrinsics:

```python
camera_matrix = [
    [600,   0, 320],
    [  0, 600, 240],
    [  0,   0,   1]
]
```

The distortion coefficients are currently assumed to be zero:

```python
dist_coeffs = np.zeros((4, 1), dtype=np.float32)
```

These values are intended as defaults for a standard webcam.

For improved accuracy, especially for precise distance estimation, the camera should ideally be calibrated and the real camera matrix and distortion coefficients should be used.

---

# ROS 2 Topics

The complete communication between the nodes is:

```text
/image_raw
     │
     ▼
TagDetectorNode
     │
     ├──────────────► /tag_pose
     │
     └──────────────► /tag_id
                          │
                          ▼
                    ChallengeNode
                          │
                          ▼
                       Robot
```

### `/image_raw`

**Type:**

```text
sensor_msgs/msg/Image
```

Camera images are received by `TagDetectorNode`.

---

### `/tag_pose`

**Type:**

```text
geometry_msgs/msg/PoseStamped
```

Contains the estimated marker position.

The position fields are used as:

```text
x = horizontal offset
y = vertical position
z = distance from camera
```

The challenge node primarily uses `x` and `z`.

---

### `/tag_id`

**Type:**

```text
std_msgs/msg/Int32
```

Contains the ID of the detected ArUco marker.

The current sorting logic is:

```text
ID 0 → Left
ID 1 → Right
```

---

# Dependencies

The project requires:

* ROS 2
* Python 3
* `rclpy`
* OpenCV
* NumPy
* `cv_bridge`
* `geometry_msgs`
* `sensor_msgs`
* `std_msgs`

The robot-specific implementation must also provide the local `Robot` class:

```python
from .robot import Robot
```

The `Robot` class is expected to provide interfaces similar to:

```python
robot.base.drive()
robot.base.stop()

robot.arm.pick_can()
robot.arm.lift()
robot.arm.place_left()
robot.arm.place_right()
robot.arm.home()
```

---

# Suggested Package Structure

A typical ROS 2 Python package could look like this:

```text
your_package/
├── package.xml
├── setup.py
├── resource/
│   └── your_package
└── your_package/
    ├── __init__.py
    ├── challenge_node.py
    ├── tag_detector_node.py
    └── robot.py
```

---

# Running the Nodes

First, source your ROS 2 installation:

```bash
source /opt/ros/<your_ros_distro>/setup.bash
```

Then build the workspace:

```bash
cd ~/your_ros2_workspace
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Start the camera/image source so that:

```text
/image_raw
```

is being published.

Then run the tag detector:

```bash
ros2 run <your_package> tag_detector_node
```

In another terminal, source the workspace again:

```bash
source ~/your_ros2_workspace/install/setup.bash
```

Then start the challenge node:

```bash
ros2 run <your_package> challenge_node
```

---

# Testing the Topics

You can check whether the camera is publishing images with:

```bash
ros2 topic echo /image_raw
```

Check the detected marker pose:

```bash
ros2 topic echo /tag_pose
```

Check the detected marker ID:

```bash
ros2 topic echo /tag_id
```

You can also inspect the available topics with:

```bash
ros2 topic list
```

---

# Important Notes

## Marker Detection

The detector currently processes the **first detected marker**:

```python
tvec = tvecs[0][0]
```

and:

```python
id_msg.data = int(ids[0][0])
```

If multiple markers are visible, the current implementation does not select between them based on distance, position, or ID.

---

## Marker Visibility

`ChallengeNode` resets:

```python
self.target_visible = False
```

at the end of every control-loop iteration.

This means the marker detector must continuously publish `/tag_pose` for the target to remain considered visible.

If the vision node stops detecting the marker, the challenge node will eventually return to the `SEARCHING` state.

---

## Camera Calibration

The current camera matrix is approximate. For a real robot, camera calibration is recommended.

An inaccurate camera matrix can result in incorrect distance estimates, which can affect:

* Approach behavior.
* Pickup distance.
* Alignment.
* Overall sorting accuracy.

---

## Coordinate Frames

The pose is published with:

```python
pose_msg.header.frame_id = "camera_frame"
```

The current implementation directly uses the camera-frame `x` and `z` values for robot control.

If the camera is mounted at an angle or offset from the robot base, additional coordinate-frame transformations may be required.

---

# Troubleshooting

### No marker is detected

Check that `/image_raw` is publishing:

```bash
ros2 topic list
ros2 topic echo /image_raw
```

Also verify that:

* The camera is working.
* The marker uses the `DICT_4X4_50` dictionary.
* The marker is clearly visible.
* The marker is sufficiently large in the image.
* Lighting is adequate.

---

### Robot keeps rotating

The robot remains in `SEARCHING` while:

```python
self.target_visible == False
```

If the detector is not publishing a valid pose, the robot will continue rotating.

Check:

```bash
ros2 topic echo /tag_pose
```

---

### Robot approaches incorrectly

The approach controller depends on:

```python
target_x_offset
target_z_dist
```

If the robot steers in the wrong direction, check the sign convention for the camera's X axis.

The angular control is:

```python
angular_speed = -self.KP_ANGULAR * self.target_x_offset
```

You may need to reverse the sign depending on the camera and robot coordinate conventions.

---

### Distance is inaccurate

The distance estimate depends on:

```python
MARKER_SIZE
camera_matrix
dist_coeffs
```

Make sure `MARKER_SIZE` exactly matches the physical marker size.

For better accuracy, calibrate the camera and replace the approximate camera parameters.

---

# Mission Summary

The complete behavior can be summarized as:

```text
1. Start robot
       ↓
2. Search for ArUco marker
       ↓
3. Detect marker
       ↓
4. Center robot on marker
       ↓
5. Drive toward marker
       ↓
6. Stop at pickup distance
       ↓
7. Pick up object
       ↓
8. Lift object
       ↓
9. Read marker ID
       ↓
10. ID 0 → Place Left
    ID 1 → Place Right
       ↓
11. Return arm to home
       ↓
12. Mission complete
```

---

# Future Improvements

Possible improvements include:

* Use proper camera calibration.
* Add visualization of detected ArUco markers.
* Handle multiple detected markers.
* Select the closest marker instead of always using the first marker.
* Add timeout handling if a marker is lost.
* Add configurable ROS 2 parameters for control gains.
* Add a dedicated `ERROR` state.
* Add emergency-stop behavior.
* Transform camera coordinates into the robot base frame using TF2.
* Prevent repeated arm actions if the control loop executes multiple times in the same state.
* Add launch files to start the camera, detector, and challenge node together.

---

# License

Add your project license here.

For example:

```text
MIT License
```

or use the license required by your course/project.
