#!/usr/bin/env python3
"""ROS2 port of databus_single.py — same serial/DAS logic as ROS1; rclpy replaces rospy."""
import logging
import os
import queue
import struct
import subprocess
import sys
import threading
import time
import traceback

import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int8MultiArray

from robot_driver_ros2.das_protocol import DASProtocol
from robot_driver_ros2.pack import CmdPack, MessagePack, Opcode, RecordType

QOS_DEFAULT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

TOPIC_LEFT_TACTILE = "/das_controller/tactile_single_l"
TOPIC_RIGHT_TACTILE = "/das_controller/tactile_single_r"
TOPIC_ENCODER = "/das_controller/encoder_data"
TOPIC_TARGET_DISTANCE = "/das_controller/target_dis"


def load_ros_params(node: Node):
    """Load topic names from node parameters (same defaults as ROS1 private ~params)."""
    global TOPIC_LEFT_TACTILE, TOPIC_RIGHT_TACTILE, TOPIC_ENCODER, TOPIC_TARGET_DISTANCE

    node.declare_parameter("topic_left_tactile", "/das_controller/tactile_single_l")
    node.declare_parameter("topic_right_tactile", "/das_controller/tactile_single_r")
    node.declare_parameter("topic_encoder", "/das_controller/encoder_data")
    node.declare_parameter("topic_target_distance", "/das_controller/target_dis")

    TOPIC_LEFT_TACTILE = node.get_parameter("topic_left_tactile").value
    TOPIC_RIGHT_TACTILE = node.get_parameter("topic_right_tactile").value
    TOPIC_ENCODER = node.get_parameter("topic_encoder").value
    TOPIC_TARGET_DISTANCE = node.get_parameter("topic_target_distance").value

    node.get_logger().info("DAS topic config loaded:")
    node.get_logger().info(f"  encoder: {TOPIC_ENCODER}")
    node.get_logger().info(f"  target distance: {TOPIC_TARGET_DISTANCE}")


class DataBus:
    def __init__(
        self,
        node: Node,
        tty_port="/dev/ttyUSB0",
        baudrate=115200,
        timeout=0.5,
        is_calib_cmd=True,
        calib_cmd_name: str = None,
        encoder_freq: float = None,
        tactile_freq: float = None,
    ):
        self.node = node
        load_ros_params(self.node)

        self.tty_port = tty_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.is_running = False

        self._open_serial_success = False
        self.protocol: DASProtocol = DASProtocol()
        self.data_buffer: bytes = b""
        self.data_buffer_lock = threading.Lock()
        self.serial_lock = threading.Lock()

        self.cmd_queue = queue.Queue(1000)

        self.read_thread: threading.Thread = None
        self.parse_thread: threading.Thread = None
        self.send_thread: threading.Thread = None

        self.encoder_freq = encoder_freq
        self.tactile_freq = tactile_freq
        self.encoder_thread: threading.Thread = None
        self.tactile_thread: threading.Thread = None

        self.gripper_dis = 0.0
        self.angle_lock = threading.Lock()
        self.is_calib_cmd = is_calib_cmd
        self.calib_cmd_name = calib_cmd_name
        self.tactile_callback = self._tactile_callback
        self.encoder_callback = self._encoder_callback
        self.echo_callback = echo_callback
        self.camera_calib_callback = camera_calib_callback

        self._tactile_pub_initialized = False
        self._encoder_pub_initialized = False

        self._init_ros_subscribers()

        self._open_serial()
        self.is_running = True
        self._start_reading()
        self._start_parsing()
        self._start_sending()

        if self.encoder_freq:
            self._start_encoder_loop()
        if self.tactile_freq:
            self._start_tactile_loop()

    def _tactile_callback(self, record_data: bytes):
        if len(record_data) != 448:
            print(f"Bad data length: expected 448 bytes, got {len(record_data)}")
            return

        if not self._tactile_pub_initialized:
            self.left_publisher_uint8 = self.node.create_publisher(
                Int8MultiArray, TOPIC_LEFT_TACTILE, QOS_DEFAULT
            )
            self.right_publisher_uint8 = self.node.create_publisher(
                Int8MultiArray, TOPIC_RIGHT_TACTILE, QOS_DEFAULT
            )
            self._tactile_pub_initialized = True

        try:
            raw_left_224 = [struct.unpack("B", record_data[i : i + 1])[0] for i in range(0, 224)]
            raw_right_224 = [struct.unpack("B", record_data[i : i + 1])[0] for i in range(224, 448)]

            left_expanded_448 = []
            for val in raw_left_224:
                left_expanded_448.append(val)
                left_expanded_448.append(val)

            right_expanded_448 = []
            for val in raw_right_224:
                right_expanded_448.append(val)
                right_expanded_448.append(val)

            total_grid = [[0 for _ in range(10)] for _ in range(100)]

            left_neg_coords = [
                (0, 0),
                (0, 1),
                (0, 2),
                (1, 0),
                (1, 1),
                (2, 0),
                (0, 7),
                (0, 8),
                (0, 9),
                (1, 8),
                (1, 9),
                (2, 9),
                (49, 0),
                (49, 1),
                (49, 2),
                (49, 3),
                (48, 0),
                (48, 1),
                (48, 2),
                (48, 3),
                (47, 0),
                (47, 1),
                (47, 2),
                (46, 0),
                (46, 1),
                (46, 2),
                (45, 0),
                (45, 1),
                (45, 2),
                (44, 0),
                (44, 1),
                (43, 0),
                (49, 6),
                (49, 7),
                (49, 8),
                (49, 9),
                (48, 6),
                (48, 7),
                (48, 8),
                (48, 9),
                (47, 7),
                (47, 8),
                (47, 9),
                (46, 7),
                (46, 8),
                (46, 9),
                (45, 7),
                (45, 8),
                (45, 9),
                (44, 8),
                (44, 9),
                (43, 9),
            ]

            right_neg_coords = [
                (50, 0),
                (50, 1),
                (50, 2),
                (51, 0),
                (51, 1),
                (52, 0),
                (50, 7),
                (50, 8),
                (50, 9),
                (51, 8),
                (51, 9),
                (52, 9),
                (99, 0),
                (99, 1),
                (99, 2),
                (99, 3),
                (98, 0),
                (98, 1),
                (98, 2),
                (98, 3),
                (97, 0),
                (97, 1),
                (97, 2),
                (96, 0),
                (96, 1),
                (96, 2),
                (95, 0),
                (95, 1),
                (95, 2),
                (94, 0),
                (94, 1),
                (93, 0),
                (99, 6),
                (99, 7),
                (99, 8),
                (99, 9),
                (98, 6),
                (98, 7),
                (98, 8),
                (98, 9),
                (97, 7),
                (97, 8),
                (97, 9),
                (96, 7),
                (96, 8),
                (96, 9),
                (95, 7),
                (95, 8),
                (95, 9),
                (94, 8),
                (94, 9),
                (93, 9),
            ]

            for (r, c) in left_neg_coords:
                total_grid[r][c] = -1
            for (r, c) in right_neg_coords:
                total_grid[r][c] = -1

            left_idx = 0
            for row in range(50):
                for col in range(10):
                    if total_grid[row][col] != -1 and left_idx < len(left_expanded_448):
                        total_grid[row][col] = left_expanded_448[left_idx]
                        left_idx += 1

            right_idx = 0
            for row in range(50, 100):
                for col in range(10):
                    if total_grid[row][col] != -1 and right_idx < len(right_expanded_448):
                        total_grid[row][col] = right_expanded_448[right_idx]
                        right_idx += 1

            left_flat = []
            for row in range(50):
                left_flat.extend(total_grid[row])

            right_flat = []
            for row in range(50, 100):
                right_flat.extend(total_grid[row])

            msg_left = Int8MultiArray()
            msg_left.data = [x if x == -1 else (x if x < 128 else x - 256) for x in left_flat]

            msg_right = Int8MultiArray()
            msg_right.data = [x if x == -1 else (x if x < 128 else x - 256) for x in right_flat]

            self.left_publisher_uint8.publish(msg_left)
            self.right_publisher_uint8.publish(msg_right)

        except Exception as e:
            print(f"Tactile processing error: {e}")
            traceback.print_exc()

    def _encoder_callback(self, record_data: bytes):
        if not self._encoder_pub_initialized:
            self.encoder_publisher = self.node.create_publisher(Float32, TOPIC_ENCODER, QOS_DEFAULT)
            self._encoder_pub_initialized = True

        self.node.get_logger().info(f"Encoder callback - Record data: {record_data}")

        encoder_value = struct.unpack(">f", record_data)[0]

        try:
            msg = Float32()
            msg.data = encoder_value
            self.node.get_logger().info(f"Encoder: {encoder_value}")
            self.encoder_publisher.publish(msg)
        except Exception as e:
            print(f"Error publishing encoder: {e}")

    def _init_ros_subscribers(self):
        self.motor_cmd_subscriber = self.node.create_subscription(
            Float32,
            TOPIC_TARGET_DISTANCE,
            self._motor_command_callback,
            QOS_DEFAULT,
        )

    def _motor_command_callback(self, msg):
        try:
            with self.angle_lock:
                self.gripper_dis = msg.data
        except Exception as e:
            print(f"Motor command handling error: {e}")

    def drive_motor(self, angle_dgree: float):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.WriteDrive,
                record_type=RecordType.Drive,
                record=struct.pack(">f", angle_dgree),
            )
        )

    def disable_motor(self):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.DisableDrive,
                record_type=RecordType.Drive,
            )
        )

    def calib_encoder(self):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.CalibEncoder,
                record_type=RecordType.Drive,
            )
        )

    def send_camera_calib_cmd(self, camera_cmd: str):
        try:
            cmd = CmdPack.pack_calib(record=camera_cmd.encode("utf-8"))
            success = self.add_cmd(cmd)
            if success:
                print(f"Sent camera calib command: {camera_cmd}")
            else:
                print(f"Failed to queue camera calib command: {camera_cmd}")
            return success
        except Exception as e:
            print(f"Error sending camera calib command: {e}")
            return False

    def add_cmd(self, cmd: CmdPack) -> bool:
        try:
            self.cmd_queue.put(cmd, block=True, timeout=1)
            return True
        except queue.Full:
            print("Command queue full, add failed")
            return False

    def is_opend(self):
        return self._open_serial_success

    def register_tactile_callback(self, callback):
        self.tactile_callback = callback

    def register_encoder_callback(self, callback):
        self.encoder_callback = callback

    def register_camera_calib_callback(self, callback):
        self.camera_calib_callback = callback

    def _open_serial(self):
        self.ser = serial.Serial(
            port=self.tty_port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        if self.ser.is_open:
            print(f"open {self.tty_port} success!, baudrate: {self.baudrate}")
            self._open_serial_success = True
        else:
            print(f"open {self.tty_port} failed!, baudrate: {self.baudrate}")
            self._open_serial_success = False

    def _start_reading(self):
        self.read_thread = threading.Thread(target=self._reading_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        return True

    def _start_parsing(self):
        self.parse_thread = threading.Thread(target=self._parsing_loop)
        self.parse_thread.daemon = True
        self.parse_thread.start()
        return True

    def _start_encoder_loop(self):
        self.encoder_thread = threading.Thread(target=self._send_encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()
        return True

    def _start_tactile_loop(self):
        self.tactile_thread = threading.Thread(target=self._send_tactile_loop)
        self.tactile_thread.daemon = True
        self.tactile_thread.start()
        return True

    def _start_sending(self):
        self.send_thread = threading.Thread(target=self._sending_loop)
        self.send_thread.daemon = True
        self.send_thread.start()
        return True

    def _sending_loop(self):
        while self.is_running and rclpy.ok():
            try:
                cmd: CmdPack = self.cmd_queue.get(block=True, timeout=0.1)
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        self.ser.write(cmd.data)
                        self.ser.flush()

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Send error: {e}")
                time.sleep(0.01)

    def _reading_loop(self):
        while self.is_running and rclpy.ok():
            try:
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        n = self.ser.inWaiting()
                        if n:
                            data = self.ser.read(n)
                            with self.data_buffer_lock:
                                self.data_buffer = self.data_buffer + data

            except Exception as e:
                print(f"Read loop error: {e}")
                time.sleep(0.1)

            time.sleep(0.001)

    def _parsing_loop(self):
        while self.is_running and rclpy.ok():
            with self.data_buffer_lock:
                if len(self.data_buffer) > 0:
                    packets, remain = DASProtocol.find_packet(self.data_buffer)
                    self.data_buffer = remain

                    for packet in packets:
                        if self.is_calib_cmd:
                            magic = DASProtocol.MAGIC
                            if (
                                self.calib_cmd_name == "MCUID"
                                and len(packet) > 2 * len(magic)
                                and packet.startswith(magic)
                                and packet.endswith(magic)
                            ):
                                middle = packet[len(magic) : -len(magic)]
                                try:
                                    print("MCUID:", middle.decode("ascii"))
                                except Exception:
                                    print("MCUID:", middle.hex())
                                self.is_calib_cmd = False
                                continue
                            camera_pack = MessagePack.unpack_camera_calib(packet)

                            if camera_pack:
                                if self.camera_calib_callback:
                                    self.camera_calib_callback(camera_pack)
                                self.is_calib_cmd = False
                        else:
                            pack = MessagePack.unpack(packet)
                            if not pack:
                                continue

                            for record in pack.records_:
                                if record.record_type == RecordType.Tactile:
                                    self.tactile_callback(record.record_data)
                                elif record.record_type == RecordType.Encoder:
                                    self.encoder_callback(record.record_data)
                                elif record.record_type == RecordType.Echo:
                                    self.echo_callback(record.record_data)
                                else:
                                    logging.error("record type:{} invalid !".format(record.record_type))

            time.sleep(0.01)

    def _send_encoder_loop(self):
        if not self.encoder_freq:
            return

        interval = 1.0 / self.encoder_freq

        while self.is_running and rclpy.ok():
            start_time = time.time()

            with self.angle_lock:
                dis_target = self.gripper_dis

            self.add_cmd(
                CmdPack.pack(
                    opcode=Opcode.ReadBatch,
                    record_type=RecordType.Encoder,
                    record=struct.pack(">f", dis_target),
                ),
            )

            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("Encoder loop thread exiting")

    def _send_tactile_loop(self):
        if not self.tactile_freq:
            return

        interval = 1.0 / self.tactile_freq
        print(f"Tactile loop started, {self.tactile_freq} Hz, interval {interval:.3f}s")

        while self.is_running and rclpy.ok():
            start_time = time.time()
            self.add_cmd(
                CmdPack.pack(
                    opcode=Opcode.ReadSingle,
                    record_type=RecordType.Tactile,
                    record=struct.pack(">f", 0.0),
                )
            )

            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("Tactile loop thread exiting")

    def stop(self):
        self.is_running = False

        threads_to_join = []
        if self.read_thread and self.read_thread.is_alive():
            threads_to_join.append(self.read_thread)
        if self.send_thread and self.send_thread.is_alive():
            threads_to_join.append(self.send_thread)
        if self.parse_thread and self.parse_thread.is_alive():
            threads_to_join.append(self.parse_thread)
        if self.encoder_thread and self.encoder_thread.is_alive():
            threads_to_join.append(self.encoder_thread)
        if self.tactile_thread and self.tactile_thread.is_alive():
            threads_to_join.append(self.tactile_thread)

        for thread in threads_to_join:
            thread.join(timeout=2)

        if self.ser and self.ser.is_open:
            self.ser.close()

    def get_serial_info(self):
        if self.ser and self.ser.is_open:
            return {
                "tty_port": self.tty_port,
                "baudrate": self.ser.baudrate,
                "bytesize": self.ser.bytesize,
                "parity": self.ser.parity,
                "stopbits": self.ser.stopbits,
                "timeout": self.ser.timeout,
                "in_waiting": self.ser.in_waiting,
            }
        return None


def echo_callback(record_data: bytes):
    print("echo data: {}".format(record_data))


def camera_calib_callback(camera_pack):
    pass


def check_and_fix_permission(port):
    if not os.path.exists(port):
        return False

    if os.access(port, os.R_OK | os.W_OK):
        return True

    print(f"Trying to fix permissions on {port}...")
    try:
        subprocess.run(["sudo", "chmod", "666", port], check=True)
        print(f"Permissions fixed: {port}")
        return True
    except subprocess.CalledProcessError:
        print(f"Permission fix failed; run manually: sudo chmod 666 {port}")
        return False


def find_serial_port(pattern="ttyUSB", max_retries=3, retry_interval=2, side=None):
    if side == "left":
        left_port = "/dev/ttyDeviceLeft"
        if os.path.exists(left_port):
            if check_and_fix_permission(left_port):
                print(f"Using left mapped device: {left_port}")
                return left_port
    elif side == "right":
        right_port = "/dev/ttyDeviceRight"
        if os.path.exists(right_port):
            if check_and_fix_permission(right_port):
                print(f"Using right mapped device: {right_port}")
                return right_port

    for attempt in range(max_retries):
        ports = list(serial.tools.list_ports.comports())

        matching_ports = []
        for port in ports:
            if pattern in port.device:
                matching_ports.append(port.device)

        matching_ports.sort(
            key=lambda x: int(x.replace(f"/dev/{pattern}", ""))
            if x.replace(f"/dev/{pattern}", "").isdigit()
            else -1
        )

        if matching_ports:
            print(f"Found serial ports: {matching_ports}")

            if side:
                print(f"Warning: no {side} mapped device, using first auto-detected port")

            for port in matching_ports:
                if check_and_fix_permission(port):
                    return port
            return matching_ports[-1]

        if attempt < max_retries - 1:
            print(f"Attempt {attempt + 1}: no serial port, retry in {retry_interval}s...")
            time.sleep(retry_interval)

    print(f"No serial port found after {max_retries} attempts")
    return None


def main():
    from rclpy.utilities import remove_ros_args

    argv = remove_ros_args(sys.argv)
    import argparse

    parser = argparse.ArgumentParser(description="DAS interface (ROS2)")
    parser.add_argument("--serial-port", type=str, default="", help="Serial port device")
    parser.add_argument("--camera-cmd", type=str, default="", help="Camera calibration command")
    parser.add_argument("--side", type=str, default="", choices=["left", "right", ""])
    args, _unknown = parser.parse_known_args(argv[1:] if len(argv) > 1 else [])

    side = args.side

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("das_ros_interface")

    node.declare_parameter("side", "")
    node.declare_parameter("serial_port", "")

    if not side:
        side = node.get_parameter("side").value or ""

    serial_port = None
    if args.serial_port and args.serial_port != "":
        serial_port = args.serial_port
        print(f"Using CLI serial port: {serial_port}")
    elif side:
        if side == "left":
            serial_port = "/dev/ttyDeviceLeft"
        elif side == "right":
            serial_port = "/dev/ttyDeviceRight"
        print(f"Using {side}-side mapped device: {serial_port}")
    else:
        sp = node.get_parameter("serial_port").value
        if sp and sp != "":
            serial_port = sp

    if not serial_port or serial_port == "":
        serial_port = find_serial_port("ttyUSB", side=side if side else None)
        if serial_port is None:
            node.destroy_node()
            rclpy.shutdown()
            return

    bus = DataBus(
        node,
        tty_port=serial_port,
        baudrate=921600,
        encoder_freq=30,
        is_calib_cmd=False,
    )

    time.sleep(1)

    if args.camera_cmd and args.camera_cmd != "":
        print(f"Sending camera calib command: {args.camera_cmd}")
        bus.send_camera_calib_cmd(args.camera_cmd)

    try:
        label = f"{side}-side" if side else "single-device"
        print(f"\nDevice initialized; {label} running (Ctrl+C to exit)...")
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
    finally:
        print("\nStopping device and closing serial...")
        bus.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete")


if __name__ == "__main__":
    main()
