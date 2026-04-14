#!/usr/bin/env python3
"""ROS2 camera node: UVC capture + sensor_msgs/Image, with v4l2 Video Capture filter and internal FPS stats."""
import glob
import os
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

QOS_IMAGE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


class CameraCaptureROS:
    def __init__(self, node: Node):
        self.node = node
        self._load_ros_params()

        self.node_name = self.node.get_name().replace("/", "_")
        if self.node_name == "_unnamed":
            self.node_name = "camera_default"

        self.cameras = []
        self.running = True
        self.image_publishers = []
        self._stats_frame_count = 0
        self._stats_publish_count = 0
        self._stats_last_time = time.time()

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._init_cameras()
        self._init_ros_publishers()

    def _load_ros_params(self):
        self.node.declare_parameter("show_preview", True)
        self.node.declare_parameter("resolutions", "640x480,320x240,800x600")
        self.node.declare_parameter("topic_base", "/camera_fisheye")
        self.node.declare_parameter("camera_count", 3)
        self.node.declare_parameter("usb_port", "")
        self.node.declare_parameter("video_0_main", "")
        self.node.declare_parameter("video_0_sec", "")
        self.node.declare_parameter("video_1_main", "")
        self.node.declare_parameter("video_1_sec", "")
        self.node.declare_parameter("video_2_main", "")
        self.node.declare_parameter("video_2_sec", "")
        self.node.declare_parameter("target_fps", 30.0)

        self.show_preview = self.node.get_parameter("show_preview").value
        resolutions_str = self.node.get_parameter("resolutions").value
        self.resolutions = []
        for res_str in str(resolutions_str).split(","):
            try:
                w, h = map(int, res_str.strip().split("x"))
                self.resolutions.append((w, h))
            except Exception:
                self.node.get_logger().warn(f"Cannot parse resolution string: {res_str}")

        if not self.resolutions:
            self.resolutions = [(640, 480), (320, 240), (800, 600)]

        self.topic_base = self.node.get_parameter("topic_base").value
        self.max_cameras = int(self.node.get_parameter("camera_count").value)
        self.usb_port = self.node.get_parameter("usb_port").value
        self.video0_main = self.node.get_parameter("video_0_main").value
        self.video0_sec = self.node.get_parameter("video_0_sec").value
        self.video1_main = self.node.get_parameter("video_1_main").value
        self.video1_sec = self.node.get_parameter("video_1_sec").value
        self.video2_main = self.node.get_parameter("video_2_main").value
        self.video2_sec = self.node.get_parameter("video_2_sec").value
        self.target_fps = float(self.node.get_parameter("target_fps").value)
        self.node.get_logger().info(
            "camera params: count=%s topic_base=%s video_0_main=%s video_0_sec=%s target_fps=%.1f"
            % (
                str(self.max_cameras),
                str(self.topic_base),
                str(self.video0_main),
                str(self.video0_sec),
                self.target_fps,
            )
        )

    def _signal_handler(self, signum, frame):
        self.node.get_logger().info(f"\nSignal {signum} received, stopping capture...")
        self.running = False

    def _get_physical_devices(self):
        try:
            result = subprocess.run(
                ["v4l2-ctl", "--list-devices"], capture_output=True, text=True
            )
            devices = []
            current_dev = ""
            device_names = {}

            for line in result.stdout.split("\n"):
                if not line.strip():
                    continue
                if ":" in line and not line.startswith("/dev/"):
                    current_dev = line.split(":")[0].strip()
                elif line.startswith("/dev/video"):
                    dev_path = line.strip()
                    if os.path.exists(dev_path):
                        devices.append(dev_path)
                        device_names[dev_path] = current_dev

            if self.usb_port and self.usb_port != "":
                usb_number = None
                try:
                    if "ttyUSB" in self.usb_port:
                        usb_number = int(self.usb_port.replace("/dev/ttyUSB", ""))
                    elif "ttyACM" in self.usb_port:
                        usb_number = int(self.usb_port.replace("/dev/ttyACM", ""))
                except Exception:
                    pass

                filtered_devices = []

                for dev in devices:
                    device_name = device_names.get(dev, "")

                    if self.usb_port in device_name or device_name in self.usb_port:
                        filtered_devices.append(dev)
                        continue

                    if usb_number is not None:
                        try:
                            video_number = int(dev.replace("/dev/video", ""))
                            if (
                                video_number == usb_number
                                or video_number == usb_number + 1
                                or video_number == usb_number - 1
                            ):
                                filtered_devices.append(dev)
                                continue
                        except Exception:
                            pass

                    try:
                        udev_cmd = ["udevadm", "info", "-q", "path", "-n", dev]
                        udev_result = subprocess.run(udev_cmd, capture_output=True, text=True)
                        udev_path = udev_result.stdout.strip()

                        if udev_path and "usb" in udev_path:
                            usb_info = udev_path.split("/")
                            for part in usb_info:
                                if "usb" in part and len(part) > 3:
                                    usb_result = subprocess.run(
                                        ["udevadm", "info", "-q", "path", "-n", self.usb_port],
                                        capture_output=True,
                                        text=True,
                                    )
                                    usb_path = usb_result.stdout.strip()

                                    if part in usb_path:
                                        filtered_devices.append(dev)
                                        self.node.get_logger().info(
                                            f"Matched video device by USB bus: {dev}"
                                        )
                                        break
                    except Exception:
                        pass

                if filtered_devices:
                    devices = filtered_devices
                    self.node.get_logger().info(f"Filtered video devices: {devices}")

            if len(devices) > self.max_cameras:
                devices = devices[: self.max_cameras]

            return sorted(list(set(devices))) if devices else sorted(glob.glob("/dev/video*"))
        except Exception:
            return sorted(glob.glob("/dev/video*"))

    def _try_reset_device(self, dev_path):
        try:
            udev_info = subprocess.run(
                ["udevadm", "info", "-q", "path", "-n", dev_path],
                capture_output=True,
                text=True,
            ).stdout.strip()

            if udev_info:
                usb_path = f"/sys{udev_info}/../reset"
                if os.path.exists(usb_path):
                    with open(usb_path, "w") as f:
                        f.write("1")
                    time.sleep(2)
                    return True
        except Exception:
            pass
        return False

    def _is_video_capture_device(self, dev_path):
        """True only for real Video Capture nodes (skip metadata-only /dev/videoX)."""
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", dev_path, "--all"], capture_output=True, text=True
            )
            info = result.stdout
            if "Format Video Capture:" in info:
                return True
            return False
        except Exception:
            return False

    def _init_camera(self, dev_path, cam_id):
        for attempt in range(3):
            try:
                if not os.path.exists(dev_path):
                    self.node.get_logger().warn(f"Device {dev_path} does not exist")
                    continue

                if not self._is_video_capture_device(dev_path):
                    self.node.get_logger().warn(
                        f"Device {dev_path} is not Video Capture (likely metadata), skip"
                    )
                    return False

                if attempt > 0:
                    self._try_reset_device(dev_path)
                    os.system(f"sudo chmod 666 {dev_path}")
                    os.system(f"sudo fuser -k {dev_path} 2>/dev/null")

                unique_cam_id = f"{self.node_name}_cam{cam_id}"
                cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
                if not cap.isOpened():
                    self.node.get_logger().warn(f"OpenCV cannot open device {dev_path}")
                    return False

                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))

                success = False
                actual_width = 0
                actual_height = 0

                for res in self.resolutions:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
                    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    if actual_width == res[0] and actual_height == res[1]:
                        success = True
                        break

                cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                actual_fps = float(cap.get(cv2.CAP_PROP_FPS))

                for _ in range(5):
                    cap.grab()
                    time.sleep(0.01)

                actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                self.node.get_logger().info(
                    "camera mode: dev=%s size=%dx%d target_fps=%.1f actual_fps=%.2f"
                    % (dev_path, actual_width, actual_height, self.target_fps, actual_fps)
                )

                window_name = f"{self.node_name}_{cam_id}_{actual_width}x{actual_height}"

                self.cameras.append(
                    {
                        "id": cam_id,
                        "unique_id": unique_cam_id,
                        "cap": cap,
                        "dev": dev_path,
                        "frame_count": 0,
                        "width": actual_width,
                        "height": actual_height,
                        "window_name": window_name,
                    }
                )
                return True

            except Exception as e:
                self.node.get_logger().error(f"Attempt #{attempt+1} init {dev_path} failed: {str(e)}")
                if "cap" in locals() and cap.isOpened():
                    cap.release()
                time.sleep(1)
        return False

    def _init_cameras(self):
        discovered = self._get_physical_devices()
        if not self.video0_main and discovered:
            self.video0_main = discovered[0]
            self.node.get_logger().info(f"video_0_main fallback to discovered {self.video0_main}")

        if self.max_cameras >= 1:
            self._init_main_or_second_camera(self.video0_main, self.video0_sec, 0)
        if self.max_cameras >= 2:
            self._init_main_or_second_camera(self.video1_main, self.video1_sec, 1)
        if self.max_cameras >= 3:
            self._init_main_or_second_camera(self.video2_main, self.video2_sec, 2)

        if not self.cameras:
            self.node.get_logger().fatal("No camera available")
            sys.exit(1)

    def _init_main_or_second_camera(self, dev_main, dev_second, index):
        if dev_main:
            self._init_camera(dev_main, index)
        if dev_second:
            self._init_camera(dev_second, index)

    def _init_ros_publishers(self):
        for cam in self.cameras:
            if self.topic_base:
                if cam["id"] == 0:
                    topic_name = f"{self.topic_base}/color/image_raw"
                else:
                    topic_name = f'{self.topic_base}_{cam["id"]}/color/image_raw'
            else:
                if cam["id"] == 0:
                    topic_name = "/camera_fisheye/color/image_raw"
                else:
                    topic_name = f'/camera_fisheye/color/image_raw_{cam["id"] + 1}'

            publisher = self.node.create_publisher(Image, topic_name, QOS_IMAGE)
            self.image_publishers.append(
                {
                    "publisher": publisher,
                    "cam_id": cam["id"],
                    "unique_id": cam["unique_id"],
                    "topic_name": topic_name,
                }
            )

    def _publish_frame(self, cam_id, frame, stamp_msg):
        try:
            if frame is None or frame.size == 0:
                return

            if len(frame.shape) != 3 or frame.shape[2] != 3:
                return

            ros_image = Image()
            ros_image.header.stamp = stamp_msg

            for cam in self.cameras:
                if cam["id"] == cam_id:
                    ros_image.header.frame_id = cam["unique_id"]
                    break

            height, width = frame.shape[:2]
            ros_image.height = height
            ros_image.width = width

            ros_image.encoding = "bgr8"
            ros_image.step = width * 3
            ros_image.is_bigendian = 0

            if not frame.flags["C_CONTIGUOUS"]:
                frame = np.ascontiguousarray(frame)

            ros_image.data = frame.tobytes()

            for pub_info in self.image_publishers:
                if pub_info["cam_id"] == cam_id:
                    pub_info["publisher"].publish(ros_image)
                    self._stats_publish_count += 1
                    break

        except Exception:
            pass

    def _display_frames(self, frames_data):
        for cam, frame in frames_data:
            if frame is not None:
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                info_text = f"{cam['unique_id']} | {timestamp} | Frames: {cam['frame_count']}"
                cv2.putText(
                    frame,
                    info_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow(cam["window_name"], frame)

        if cv2.waitKey(1) == 27:
            self.running = False

    def capture_frames(self):
        if self.show_preview:
            for cam in self.cameras:
                RESIZE_WIDTH = 640
                RESIZE_HEIGHT = 480
                cv2.namedWindow(cam["window_name"], cv2.WINDOW_NORMAL)
                cv2.resizeWindow(cam["window_name"], RESIZE_WIDTH, RESIZE_HEIGHT)

        frame_num = 0
        period = 1.0 / 30.0

        try:
            while self.running and rclpy.ok():
                t0 = time.time()
                stamp = self.node.get_clock().now().to_msg()
                frames_data = []

                for cam in self.cameras:
                    ret, frame = cam["cap"].read()
                    if not ret:
                        frame = None
                    else:
                        self._publish_frame(cam["id"], frame, stamp)
                        cam["frame_count"] += 1

                    frames_data.append((cam, frame))

                if self.show_preview:
                    self._display_frames(frames_data)

                frame_num += 1
                self._stats_frame_count += len(self.cameras)

                now = time.time()
                if now - self._stats_last_time >= 2.0:
                    dt = now - self._stats_last_time
                    cam_fps = self._stats_frame_count / dt if dt > 0 else 0.0
                    pub_fps = self._stats_publish_count / dt if dt > 0 else 0.0
                    self.node.get_logger().info(
                        "camera internal fps: capture=%.2f publish=%.2f cameras=%d"
                        % (cam_fps, pub_fps, len(self.cameras))
                    )
                    self._stats_frame_count = 0
                    self._stats_publish_count = 0
                    self._stats_last_time = now

                elapsed = time.time() - t0
                time.sleep(max(0, period - elapsed))

        except Exception as e:
            self.node.get_logger().error(f"Capture error: {e}")
        finally:
            self._release_resources()

    def _release_resources(self):
        for cam in self.cameras:
            try:
                cam["cap"].release()
            except Exception:
                pass

        if self.show_preview:
            for cam in self.cameras:
                try:
                    cv2.destroyWindow(cam["window_name"])
                except Exception:
                    pass


def main():
    try:
        os.nice(-20)
    except Exception:
        pass

    cv2.setNumThreads(1)
    cv2.setUseOptimized(True)

    from argparse import ArgumentParser

    from rclpy.utilities import remove_ros_args

    argv_user = remove_ros_args(sys.argv)
    parser = ArgumentParser(description="Camera capture script with ROS2")
    parser.add_argument(
        "--no-preview", dest="show_preview", action="store_false", help="Disable preview window"
    )
    parser.add_argument("--usb-port", type=str, default="", help="USB port for filtering video devices")
    parser.add_argument("--video_0_main", type=str, default="", help="center video devices")
    parser.add_argument("--video_1_main", type=str, default="", help="video devices")
    parser.add_argument("--video_2_main", type=str, default="", help="video devices")
    parser.add_argument("--video_0_sec", type=str, default="", help="center video devices")
    parser.add_argument("--video_1_sec", type=str, default="", help="left video devices")
    parser.add_argument("--video_2_sec", type=str, default="", help="video devices")
    parser.set_defaults(show_preview=True)
    parser.parse_known_args(argv_user[1:] if len(argv_user) > 1 else [])

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("camera_capture_node")

    try:
        recorder = CameraCaptureROS(node)
        recorder.capture_frames()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {str(e)}")
        print("\nRecovery hints:")
        print("1. Power-cycle the camera")
        print("2. sudo rmmod uvcvideo && sudo modprobe uvcvideo")
        print("3. Try another USB port")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
