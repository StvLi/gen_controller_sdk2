# gen_controller_sdk2 (ROS2 Foxy)

This directory is the ROS2 counterpart of ROS1 `gen_controller_sdk_release`.
The core package is `robot_driver_ros2`.

Goal: provide a directly usable gripper camera/tactile/encoder communication package on Ubuntu 20.04 + ROS2 Foxy, while keeping topic naming and launch usage as close to ROS1 as possible.

## 1. Directory Structure

```text
gen_controller_sdk2/
└── robot_driver_ros2/
    ├── launch/
    │   ├── single_gripper_start.launch.xml
    │   └── dual_gripper_start.launch.xml
    ├── config/99-usb-serial.rules
    ├── robot_driver_ros2/
    │   ├── camera_view_single.py
    │   ├── databus_single.py
    │   ├── tactile_dual_print.py
    │   ├── left_das_controller_infer.py
    │   └── right_das_controller_infer.py
    └── package.xml
```

## 2. Environment Requirements

- Ubuntu 20.04
- ROS2 Foxy
- Python 3.8 (default Python version for Foxy)
- USB 3.0 (recommended for camera throughput)

Install system dependencies:

```bash
sudo apt update
sudo apt install -y python3-serial python3-opencv v4l-utils
```

Optional Python dependencies:

```bash
pip install -r robot_driver_ros2/requirements.txt
```

## 3. udev Device Mapping

Template file:

```text
robot_driver_ros2/config/99-usb-serial.rules
```

Install and reload rules:

```bash
sudo cp robot_driver_ros2/config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Recommended device names to verify:

- Serial: `/dev/ttyDeviceLeft`, `/dev/ttyDeviceRight`
- Video: `/dev/left_video_0_main`, `/dev/left_video_0_sec` (same pattern for right side)

## 4. Build and Source

Run in the root of `das_ros2_ws`:

```bash
cd /home/stvli/Desktop/das_controller_test/das_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select robot_driver_ros2 --symlink-install
source install/setup.bash
```

## 5. Launch

### 5.1 Single Gripper (Camera + DAS)

```bash
ros2 launch robot_driver_ros2 single_gripper_start.launch.xml
```

### 5.2 Dual Gripper (Left + Right)

```bash
ros2 launch robot_driver_ros2 dual_gripper_start.launch.xml
```

### 5.3 Override Parameters Example

```bash
ros2 launch robot_driver_ros2 single_gripper_start.launch.xml \
  serial:=/dev/ttyDeviceLeft \
  camera_resolutions:=1600x1296 \
  show_preview:=false
```

## 6. Main Topics

Common single-gripper outputs:

- `/camera/color/image_raw`
- `/encoder`
- `/tactile/left`
- `/tactile/right`
- `/target_distance` (control input)

Common dual-gripper outputs:

- `/left_gripper/camera/color/image_raw`
- `/left_gripper/encoder`
- `/left_gripper/tactile/left`
- `/left_gripper/tactile/right`
- `/left_gripper/target_distance`
- `/right_gripper/...` (same structure)

## 7. Common Debug Commands

Check image topic frequency:

```bash
ros2 topic hz /camera/color/image_raw
```

Tactile print:

```bash
# Single gripper
ros2 run robot_driver_ros2 tactile_dual_print

# Left side in dual gripper mode
ros2 run robot_driver_ros2 tactile_dual_print --ros-args -p gripper_ns:=left_gripper
```

Inference bridge nodes (optional):

```bash
ros2 run robot_driver_ros2 left_das_controller_infer
ros2 run robot_driver_ros2 right_das_controller_infer
```

## 8. Important Notes

- `camera_view_single` and `databus_single` are launched together in launch files. For camera-only debugging, run `ros2 run robot_driver_ros2 camera_view_single`.
- If you hit ROS1/ROS2 mixed-environment issues, open a new terminal and only run:
  - `source /opt/ros/foxy/setup.bash`
  - `source <das_ros2_ws>/install/setup.bash`
- This package currently does not provide a runnable `camera_calib_cli` entry point; `scripts/camera_cmd.sh` depends on it. Add that node or adjust the script before use.

## 9. Current Foxy Test Status

- **Functionality**: single/dual launch, camera publish, encoder/tactile publish, and target_distance command path are available.
- **Known performance gap**: at high resolution (for example `1600x1296`), image topic frequency in Foxy is often lower than ROS1 on the same machine.
- **Observed range during debugging**: ROS2 image topic has shown values around `~16 Hz` in stable runs, and lower in stressed configurations.
- **Stability note**: this repository currently keeps a usable Foxy baseline and avoids aggressive architecture changes in Foxy.

## 10. Improvement Plan

- **Short term (current Foxy branch)**:
  - Keep interfaces and launch behavior stable.
  - Focus on reproducible testing and issue documentation rather than deep transport rework.
- **Next phase (Humble/Jazzy migration)**:
  - Re-run the same test matrix (resolution, preview on/off, single/dual node).
  - Re-evaluate image throughput and latency on newer ROS2 distributions.
  - If needed, move performance-critical image path to C++ components and intra-process optimization.
- **Documentation-first policy**:
  - Record each benchmark result (topic hz + internal capture/publish stats) before any optimization commit.

## 11. Reference Documents

- ROS2 usage guide: `/home/stvli/Desktop/das_controller_test/das_ros2_ws/docs/ROBOT_DRIVER_ROS2_USAGE.md`
- Migration pitfalls: `/home/stvli/Desktop/das_controller_test/docs/pitfalls/GEN_CONTROLLER_ROS2_PITFALLS.md`
