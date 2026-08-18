# Trash Sorting Robot - Project Guide

Welcome to the project. Your robot's job: search for a tagged trash
item, approach it, pick it up, identify which category it belongs to,
and place it in the correct bin.

The vision system (tag detection + category classification) is already
finished and handed to you as-is - you shouldn't need to modify it. The
rest of this repo (last week's competition code) is given to you as a
**reference implementation**, not something to copy wholesale. You'll
be building your own version, in your own repo, with your own package.
Understanding *why* each piece works matters more than having working
code you didn't write yourselves - you'll be tuning gains, adding
poses, and extending the sorting logic for new categories, and that's
much harder to do with code you don't understand.

## 1. Creating your ROS2 package

Start a workspace and package from scratch:

```bash
mkdir -p ~/trash_sorter_ws/src
cd ~/trash_sorter_ws/src

ros2 pkg create --build-type ament_python trash_sorter \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs cv_bridge
```

This creates:

```
trash_sorter/
├── trash_sorter/          <- your Python source files go here
│   └── __init__.py
├── resource/
│   └── trash_sorter
├── package.xml
├── setup.py
└── setup.cfg
```

Every node you write goes inside the inner `trash_sorter/` folder
(this is a common source of confusion - there are two folders with the
same name, the outer one is the whole package, the inner one is the
actual Python module).

### Registering your nodes

Every node needs an entry in `setup.py` so `ros2 run` can find it.
Open `setup.py` and add each node under `entry_points`:

```python
entry_points={
    'console_scripts': [
        'challenge_node = trash_sorter.challenge_node:main',
        'tag_detector_node = trash_sorter.tag_detector_node:main',
        'crane_driver_node = trash_sorter.crane_driver_node:main',
        'gui = trash_sorter.gui:main',
    ],
},
```

Whenever you add a new node file with its own `main()`, add a matching
line here - if you forget this step, `ros2 run` won't find your node
even though the file exists.

### Building

From the workspace root (not inside `src/`):

```bash
cd ~/trash_sorter_ws
colcon build
source install/setup.bash
```

You'll need to `source install/setup.bash` again in every new terminal
you open (or add it to your `~/.bashrc`).

## 2. Package structure - what goes where

| File | Purpose | Status |
|------|---------|--------|
| `tag_detector_node.py` | Vision node - detects tags, publishes `/tag_pose` and `/tag_id` | **Given, finished.** Copy from the old repo as-is. |
| `crane_driver_node.py` | Low-level driver - talks directly to the Dynamixel servos over serial | **Given, hardware-specific.** Reference/reuse as-is unless your hardware differs. |
| `arm.py` | High-level arm interface (`pick_can()`, `place_left()`, etc.) - wraps raw joint commands into named actions | **Reference.** You'll extend this with new actions for your categories/bins. |
| `base.py` | High-level base interface (`drive()`, `forward()`, `left()`, etc.) | **Reference.** Should mostly work as-is; understand it before touching it. |
| `robot.py` | Thin wrapper that just bundles `ArmController` + `BaseController` together | **Reference.** Rarely needs changes. |
| `poses.py` | Predefined joint-angle poses (in degrees) that `arm.py`'s actions move through | **You will rewrite this.** Last week's poses were tuned for 2 categories/2 bins. You have 3 (+1 stretch), so you need new placement poses for the new bin layout - see Section 5. |
| `challenge_node.py` | The main state machine - SEARCHING → APPROACHING → PICKING → SORTING → FINISHED | **Skeleton with TODOs**, provided separately - see the project task list, not this file. |
| `gui.py` | PyQt5 control panel with joint sliders and base movement buttons | **Debug tool only** - not part of the competition run. Use this to manually jog the arm and find new pose values for `poses.py`, and to test base movement in isolation. It does not contain any competition logic itself. |

## 3. Running the system

Launch order matters - the camera and driver need to be up before
anything that depends on them.

**Terminal 1 - camera:**
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video2 \
  -p pixel_format:=YUYV \
  -p image_size:="[320,240]"
```

**Terminal 2 - arm driver:**
```bash
ros2 run trash_sorter crane_driver_node
```

**Terminal 3 - tag detector:**
```bash
ros2 run trash_sorter tag_detector_node
```

**Terminal 4 - either the competition logic OR the debug GUI, not both:**
```bash
ros2 run trash_sorter challenge_node
```
```bash
# OR, for manual pose tuning:
ros2 run trash_sorter gui
```
Don't run `challenge_node` and `gui` at the same time - both create a
`Robot` (arm + base) and will publish conflicting commands to the same
topics if run together.

## 4. Category mapping (from the vision system)

| Tag ID | Category | Japanese |
|--------|----------|----------|
| 0 | Burnable | 可燃ごみ (kanenengomi) |
| 1 | PET Bottle | ペットボトル (petbotoru) |
| 2 | Can | 缶 (kan) |
| 3 | Non-burnable *(stretch goal)* | 不燃ごみ (funenengomi) |

`/tag_id` publishes this as a raw int - your `challenge_node`'s SORTING
state is responsible for mapping each ID to an arm action and bin.

## 5. Generating and printing tags

```bash
python3 generate_tags.py
```

Produces one PNG per category in `tags_output/`. Print each **at 100%
scale / "actual size"** - never "fit to page". After printing, measure
the black square with a ruler; it must be exactly 50mm. If it's off,
either reprint at the correct scale or update `MARKER_SIZE` in
`tag_detector_node.py` to match your actual printed size - a mismatch
here biases every distance reading by the same proportion.

## 6. Finding new arm poses

Since you have more categories than last week, you'll need new
placement poses (and possibly a new pick sequence if your item shape
differs). Use `gui.py` for this:

1. Run the GUI (`ros2 run trash_sorter gui`), with the arm powered and
   the driver node running.
2. Use the joint sliders to manually move the arm to the position you
   want (e.g. hovering over a new bin).
3. Read off the resulting joint angles from the value labels next to
   each slider.
4. Copy those degree values into a new list in `poses.py`, following
   the existing style (base, shoulder, elbow, wrist, gripper).
5. Add a corresponding method in `arm.py` (e.g. `place_center()`) that
   moves through your new pose, following the pattern of
   `place_left()`/`place_right()`.

## 7. Troubleshooting

**Detection is slow or drops out for seconds at a time.**
Check the actual publish rate:
```bash
ros2 topic hz /tag_pose
```
If this is far below the camera's frame rate, the most common cause is
a QoS mismatch between the camera driver and the detector's
subscription (already handled in the given `tag_detector_node.py`, but
worth re-checking if you modify anything camera-related).

**Something doesn't seem to be receiving messages / nodes aren't talking to each other.**
```bash
rqt_graph
```
This draws a live graph of every running node and the topics
connecting them. It's the fastest way to spot an obvious wiring
problem - a node that's supposed to be subscribed to something but
isn't connected, a typo'd topic name, or a node that isn't running at
all. Check this first before diving into code when something seems to
"just not respond."

**You're not sure if the camera itself is working, independent of detection.**
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/image_raw` from the topic dropdown to view the raw camera
feed directly, bypassing the detector entirely. Useful for isolating
"is this a camera problem or a detection-logic problem" - if the raw
feed looks fine here but detection still isn't working, the issue is
downstream in `tag_detector_node.py` or your parameters, not the
camera.

**Distances/offsets look wrong or jump around unpredictably.**
1. Confirm your printed marker is exactly 50mm (see Section 5).
2. Check the log line printed on detector startup - it reports what
   resolution the camera matrix was initialized for. Confirm it
   matches your camera's actual output.
3. If two different tags are visible in the same frame at once, the
   detector always targets the closer one - use the detector's debug
   window (or `rqt_image_view`) to confirm that's actually what's
   happening rather than a false detection.

**An unexpected/wrong category ID shows up.**
The detector logs every raw detected ID (throttled to 2/sec), even IDs
outside the known category list - watch this log to see if it's
picking up something unintended (a QR code, glare, or unrelated
pattern in the background). Only IDs 0-3 are ever published to
`/tag_id`; anything else is logged but ignored.