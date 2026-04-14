#!/usr/bin/env python3
"""ROS2 port of left_das_controller_infer.py — same topic mapping as ROS1."""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

QOS_DEFAULT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)


class GripperDataConverter:
    def __init__(self, node: Node):
        self.node = node
        self.publish_rate = 100
        self.latest_left_data = None
        self.latest_left_cmd = None

        node.create_subscription(Float32, "/left_gripper/encoder", self.left_gripper_data_callback, QOS_DEFAULT)
        self.left_gripper_feedback_pub = node.create_publisher(
            PoseStamped, "/gripper/left/current_distance", QOS_DEFAULT
        )

        node.create_subscription(
            PoseStamped, "/target_gripper/left_gripper", self.left_cmd_callback, QOS_DEFAULT
        )
        self.left_gripper_cmd_pub = node.create_publisher(
            Float32, "/left_gripper/target_distance", QOS_DEFAULT
        )

        self.node.get_logger().info("Gripper Data Converter Node Started")
        self.node.get_logger().info(f"Publish rate: {self.publish_rate}Hz")

    def left_gripper_data_callback(self, msg):
        self.latest_left_data = msg

    def process_gripper_feedback(self, gripper_msg, publisher, gripper_name):
        pose_msg = PoseStamped()

        if hasattr(gripper_msg, "header") and gripper_msg.header.stamp.sec != 0:
            pose_msg.header.stamp = gripper_msg.header.stamp
        else:
            pose_msg.header.stamp = self.node.get_clock().now().to_msg()

        pose_msg.header.frame_id = f"{gripper_name}_gripper_frame"

        pose_msg.pose.position.x = float(gripper_msg.data)
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        publisher.publish(pose_msg)

        self.node.get_logger().info(f"Published {gripper_name} feedback distance: {gripper_msg.data}")

    def left_cmd_callback(self, msg):
        self.latest_left_cmd = msg

    def process_gripper_cmd(self, pos_msg, publisher, gripper_name):
        gripper_cmd_msg = Float32()
        gripper_cmd_msg.data = float(pos_msg.pose.position.x)
        publisher.publish(gripper_cmd_msg)

    def publish_all_data(self):
        if self.latest_left_data is not None:
            self.process_gripper_feedback(
                self.latest_left_data, self.left_gripper_feedback_pub, "left"
            )

        if self.latest_left_cmd is not None:
            self.process_gripper_cmd(self.latest_left_cmd, self.left_gripper_cmd_pub, "left")

    def run(self):
        rate = self.node.create_rate(self.publish_rate)
        while rclpy.ok():
            self.publish_all_data()
            rate.sleep()


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("das_controller_converter")
    try:
        converter = GripperDataConverter(node)
        converter.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
