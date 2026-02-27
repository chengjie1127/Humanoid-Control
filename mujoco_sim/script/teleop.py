#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading
import time

class KeyboardController:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 1)
        self.twist_msg = Twist()

    def on_press(self, key):
        try:
            if key.char == 'z':
                # Shutdown signal can be handled gracefully
                pass
            else:
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

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("keyboard_control")

    controller = KeyboardController(node)

    def ros_publish():
        while rclpy.ok():
            try:
                controller.publisher.publish(controller.twist_msg)
                time.sleep(1.0 / 150.0)
            except Exception:
                break

    thread = threading.Thread(target=ros_publish)
    thread.start()
    
    def on_press(key):
        controller.on_press(key)

    listener = keyboard.Listener(on_press=on_press, on_release=controller.on_release)
    listener.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    listener.stop()
    listener.join()
    
    node.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()