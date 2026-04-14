# gen_controller_sdk2 (ROS2 Foxy)

本目录是与 ROS1 `gen_controller_sdk_release` 对应的 ROS2 版本，核心功能包为 `robot_driver_ros2`。

目标：在 Ubuntu 20.04 + ROS2 Foxy 下提供可直接使用的夹爪相机/触觉/编码器通信能力，并保持与 ROS1 话题和 launch 习惯尽量一致。

## 1. 目录结构

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

## 2. 环境要求

- Ubuntu 20.04
- ROS2 Foxy
- Python 3.8（Foxy 默认 Python）
- USB 3.0（相机建议）

安装系统依赖：

```bash
sudo apt update
sudo apt install -y python3-serial python3-opencv v4l-utils
```

可选 Python 依赖：

```bash
pip install -r robot_driver_ros2/requirements.txt
```

## 3. udev 设备映射配置

模板文件：

```text
robot_driver_ros2/config/99-usb-serial.rules
```

安装方式：

```bash
sudo cp robot_driver_ros2/config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

建议确认以下设备名存在：

- 串口：`/dev/ttyDeviceLeft`、`/dev/ttyDeviceRight`
- 视频：`/dev/left_video_0_main`、`/dev/left_video_0_sec`（右侧同理）

## 4. 编译与加载环境

在 `das_ros2_ws` 根目录执行：

```bash
cd /home/stvli/Desktop/das_controller_test/das_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select robot_driver_ros2 --symlink-install
source install/setup.bash
```

## 5. 启动方式

### 5.1 单夹爪启动（相机 + DAS）

```bash
ros2 launch robot_driver_ros2 single_gripper_start.launch.xml
```

### 5.2 双夹爪启动（左右各一套）

```bash
ros2 launch robot_driver_ros2 dual_gripper_start.launch.xml
```

### 5.3 覆盖参数示例

```bash
ros2 launch robot_driver_ros2 single_gripper_start.launch.xml \
  serial:=/dev/ttyDeviceLeft \
  camera_resolutions:=1600x1296 \
  show_preview:=false
```

## 6. 主要话题

单夹爪常见输出：

- `/camera/color/image_raw`
- `/encoder`
- `/tactile/left`
- `/tactile/right`
- `/target_distance`（控制输入）

双夹爪常见输出：

- `/left_gripper/camera/color/image_raw`
- `/left_gripper/encoder`
- `/left_gripper/tactile/left`
- `/left_gripper/tactile/right`
- `/left_gripper/target_distance`
- `/right_gripper/...`（同结构）

## 7. 常用调试命令

查看图像频率：

```bash
ros2 topic hz /camera/color/image_raw
```

触觉打印：

```bash
# 单夹爪
ros2 run robot_driver_ros2 tactile_dual_print

# 双夹爪左侧
ros2 run robot_driver_ros2 tactile_dual_print --ros-args -p gripper_ns:=left_gripper
```

推断桥接节点（可选）：

```bash
ros2 run robot_driver_ros2 left_das_controller_infer
ros2 run robot_driver_ros2 right_das_controller_infer
```

## 8. 注意事项（务必）

- `camera_view_single` 与 `databus_single` 在 launch 内已一起启动；只调相机时可单独 `ros2 run robot_driver_ros2 camera_view_single`。
- 若出现 ROS1/ROS2 混环境异常，请使用新终端，仅执行：
  - `source /opt/ros/foxy/setup.bash`
  - `source <das_ros2_ws>/install/setup.bash`
- 当前包不提供可直接运行的 `camera_calib_cli` 入口；`scripts/camera_cmd.sh` 依赖该入口，使用前需先补齐对应节点或改造脚本。

## 9. 当前 Foxy 测试结果

- **功能可用性**：单夹爪/双夹爪 launch、相机发布、编码器/触觉发布、`target_distance` 控制链路均可工作。
- **已知性能差距**：在高分辨率（如 `1600x1296`）下，Foxy 的图像话题频率通常低于同机 ROS1。
- **调试期观测范围**：ROS2 图像话题在稳定场景中曾观测到 `~16 Hz`，高负载或不利配置下可能更低。
- **稳定性策略**：当前仓库先维持“可用且可复现”的 Foxy 基线版本，不在 Foxy 上做激进架构调整。

## 10. 后续改进计划

- **短期（当前 Foxy 分支）**：
  - 保持接口与 launch 行为稳定；
  - 优先做可复现实验与问题记录，不进行重度传输层重构。
- **下一阶段（迁移 Humble/Jazzy）**：
  - 在新环境复跑同一测试矩阵（分辨率、预览开关、单/双节点）；
  - 重新评估图像吞吐与时延；
  - 必要时将性能关键路径迁移到 C++ 组件与进程内优化方案。
- **文档先行**：
  - 每次优化前后都记录基准（`topic hz` + 节点内部 capture/publish 统计），确保结论可追溯。

## 11. 参考文档

- ROS2 使用说明：`/home/stvli/Desktop/das_controller_test/das_ros2_ws/docs/ROBOT_DRIVER_ROS2_USAGE.md`
- 迁移踩坑记录：`/home/stvli/Desktop/das_controller_test/docs/pitfalls/GEN_CONTROLLER_ROS2_PITFALLS.md`
