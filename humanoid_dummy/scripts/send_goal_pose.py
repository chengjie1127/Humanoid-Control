#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def yaw_to_quaternion(yaw_rad: float):
    half = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half), math.cos(half)


def parse_args():
    parser = argparse.ArgumentParser(description="Publish a single /goal_pose from x, y, z, yaw.")
    parser.add_argument("--x", type=float, required=True, help="Target x in the chosen frame.")
    parser.add_argument("--y", type=float, required=True, help="Target y in the chosen frame.")
    parser.add_argument("--z", type=float, required=True, help="Target z in the chosen frame.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle. Radians by default, or degrees with --degrees.")
    parser.add_argument("--degrees", action="store_true", help="Interpret --yaw as degrees.")
    parser.add_argument("--frame-id", default="odom", help="Frame id for the goal pose. Default: odom")
    parser.add_argument("--topic", default="/goal_pose", help="Topic to publish. Default: /goal_pose")
    return parser.parse_args()


class GoalPosePublisher(Node):
    def __init__(self, topic: str):
        super().__init__("send_goal_pose")
        self.publisher = self.create_publisher(PoseStamped, topic, 1)


def main():
    args = parse_args()
    yaw = math.radians(args.yaw) if args.degrees else args.yaw

    rclpy.init()
    node = GoalPosePublisher(args.topic)

    msg = PoseStamped()
    msg.header.frame_id = args.frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = args.z
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    time.sleep(0.3)
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        node.publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    node.get_logger().info(
        f"Published goal pose to {args.topic}: x={args.x:.3f} y={args.y:.3f} z={args.z:.3f} yaw={yaw:.3f} rad"
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
