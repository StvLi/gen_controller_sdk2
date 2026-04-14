#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 port of tactile_dual_print.py — same print logic as ROS1."""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8MultiArray

COLS = 10
ROWS = 50
GAP = " " * 10

QOS_SUB = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)


def _normalize_ns(ns):
    ns = (ns or "").strip()
    return ns.strip("/")


def _topic(ns, tactile_name):
    n = _normalize_ns(ns)
    return "/{}/tactile/{}".format(n, tactile_name)


class TactileDualPrinter:
    def __init__(self, node: Node):
        self.node = node
        node.declare_parameter("gripper_ns", "left_gripper")
        node.declare_parameter("print_hz", 30.0)

        gripper_ns = node.get_parameter("gripper_ns").value
        hz = float(node.get_parameter("print_hz").value)

        self._topic_left = _topic(gripper_ns, "left")
        self._topic_right = _topic(gripper_ns, "right")

        self._lock = threading.Lock()
        self._data_left = None
        self._data_right = None
        self._last_warn = 0.0

        node.get_logger().info("gripper_ns=%s" % gripper_ns)
        node.get_logger().info("subscribe tactile/left:  %s" % self._topic_left)
        node.get_logger().info("subscribe tactile/right: %s" % self._topic_right)

        node.create_subscription(Int8MultiArray, self._topic_left, self._cb_left, QOS_SUB)
        node.create_subscription(Int8MultiArray, self._topic_right, self._cb_right, QOS_SUB)

        period = max(0.02, 1.0 / hz) if hz > 0 else 0.05
        self._timer = node.create_timer(period, self._on_timer)

    def _cb_left(self, msg):
        with self._lock:
            self._data_left = list(msg.data)

    def _cb_right(self, msg):
        with self._lock:
            self._data_right = list(msg.data)

    def _on_timer(self):
        with self._lock:
            if self._data_left is None or self._data_right is None:
                return
            L, R = list(self._data_left), list(self._data_right)
        n = min(len(L), len(R), ROWS * COLS)
        if n < ROWS * COLS:
            now = time.time()
            if now - self._last_warn >= 5.0:
                self._last_warn = now
                self.node.get_logger().warn(
                    "tactile length is less than 500: left=%d right=%d, will only print the first %d numbers"
                    % (len(L), len(R), n)
                )
        for row in range(ROWS):
            i0 = row * COLS
            if i0 + COLS > n:
                break
            left_seg = L[i0 : i0 + COLS]
            right_seg = R[i0 : i0 + COLS]
            left_s = " ".join("{:3d}".format(int(x)) for x in left_seg)
            right_s = " ".join("{:3d}".format(int(x)) for x in right_seg)
            print(left_s + GAP + right_s)
        print("", flush=True)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("tactile_dual_print")
    TactileDualPrinter(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
