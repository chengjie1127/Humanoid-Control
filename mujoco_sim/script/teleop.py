#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
from rclpy.node import Node


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist_msg = Twist()
        self.create_timer(1.0 / 150.0, self.publish_cmd)

    def on_press(self, key):
        try:
            if key.char == 'z':
                rclpy.shutdown()
                return

            if key.char == 'w':
                self.twist_msg.linear.x = 0.4
            elif key.char == 's':
                self.twist_msg.linear.x = -0.4
            else:
                self.twist_msg.linear.x = 0.0

            if key.char == 'a':
                self.twist_msg.linear.y = 0.2
            elif key.char == 'd':
                self.twist_msg.linear.y = -0.2
            else:
                self.twist_msg.linear.y = 0.0

            if key.char == 'q':
                self.twist_msg.angular.z = 0.4
            elif key.char == 'e':
                self.twist_msg.angular.z = -0.4
            else:
                self.twist_msg.angular.z = 0.0
        except AttributeError:
            pass

    def on_release(self, key):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.angular.z = 0.0

    def publish_cmd(self):
        self.publisher.publish(self.twist_msg)


def main():
    rclpy.init()
    node = KeyboardController()

    listener = keyboard.Listener(on_press=node.on_press, on_release=node.on_release)
    listener.start()

    try:
        rclpy.spin(node)
    finally:
        listener.stop()
        listener.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
