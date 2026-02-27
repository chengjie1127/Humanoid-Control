#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import sys

x_data = []
y_data = []

class TorqueListener(Node):
    def __init__(self, index):
        super().__init__('torque_listener')
        self.index = index
        self.subscription = self.create_subscription(
            Float32MultiArray, '/realTorque', self.callback, 10)
            
    def callback(self, data):
        global x_data, y_data
        real_torque = data.data[self.index]
        x_data.append(self.get_clock().now().nanoseconds / 1e9)
        y_data.append(real_torque)
        
        plt.cla()
        if self.index >= 0 and self.index < 12:
            plt.plot(x_data, y_data)
            plt.xlabel('Time')
            plt.ylabel('Torque')
            plt.title(f'Torque {self.index+1}')
            plt.draw()
            plt.pause(0.001)

def main(args=None):
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        print('Please provide a valid integer argument as an index.')
        sys.exit(1)
        
    rclpy.init(args=args)
    index = int(sys.argv[1]) - 1
    
    plt.ion()
    plt.show()
    
    node = TorqueListener(index)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()