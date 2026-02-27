#!/usr/bin/env python3

import sys

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class TorqueListener(Node):
    def __init__(self, index: int):
        super().__init__('torque_listener')
        self.index = index
        self.x_data = []
        self.y_data = []
        self.create_subscription(Float32MultiArray, '/realTorque', self.callback, 10)

    def callback(self, data: Float32MultiArray):
        if self.index < 0 or self.index >= min(12, len(data.data)):
            return

        real_torque = data.data[self.index]
        now = self.get_clock().now().nanoseconds / 1e9

        self.x_data.append(now)
        self.y_data.append(real_torque)

        plt.cla()
        plt.plot(self.x_data, self.y_data)
        plt.xlabel('Time')
        plt.ylabel('Torque')
        plt.title(f'Torque {self.index + 1}')
        plt.draw()
        plt.pause(0.001)


def main():
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        print('Please provide a valid integer argument as index.')
        sys.exit(1)

    index = int(sys.argv[1]) - 1

    rclpy.init()
    node = TorqueListener(index)

    plt.ion()
    plt.show()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
