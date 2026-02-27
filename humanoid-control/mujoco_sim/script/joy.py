#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class JoyCommandNode(Node):
    def __init__(self):
        super().__init__('joy11')
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.key_y_pressed = False
        self.pre_key_y_pressed = False
        self.key_x_pressed = False
        self.pre_key_x_pressed = False
        self.hw_switch_bool = False
        self.gait_str = 'trot'

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.hw_switch_publisher = self.create_publisher(Bool, '/hwswitch', 1)
        self.gait_str_publisher = self.create_publisher(String, '/desired_gait_str', 1)

        self.create_subscription(Joy, 'joy', self.callback, 10)
        self.create_timer(1.0 / 150.0, self.publish_loop)

    def callback(self, data: Joy):
        self.vx = data.axes[1] * 0.5
        self.vy = data.axes[0] * 0.25
        self.w = data.axes[3] * 0.4
        self.key_y_pressed = data.buttons[3] == 1
        self.key_x_pressed = data.buttons[2] == 1

    def publish_loop(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.vx
        twist_msg.linear.y = self.vy
        twist_msg.angular.z = self.w
        self.publisher.publish(twist_msg)

        if self.key_y_pressed and not self.pre_key_y_pressed:
            self.hw_switch_bool = not self.hw_switch_bool
            self.get_logger().info(f'Switch the output status to {self.hw_switch_bool}')
        self.pre_key_y_pressed = self.key_y_pressed

        hw_switch_msg = Bool()
        hw_switch_msg.data = self.hw_switch_bool
        self.hw_switch_publisher.publish(hw_switch_msg)

        if self.key_x_pressed and not self.pre_key_x_pressed:
            self.gait_str = 'walk' if self.gait_str == 'trot' else 'trot'
            self.get_logger().info(f'Switch the desire gait to {self.gait_str}')
        self.pre_key_x_pressed = self.key_x_pressed

        gait_str_msg = String()
        gait_str_msg.data = self.gait_str
        self.gait_str_publisher.publish(gait_str_msg)


def main():
    rclpy.init()
    node = JoyCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
