#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
from std_msgs.msg import Bool
from std_msgs.msg import String
import time

vx = 0.0
vy = 0.0
w = 0.0
key_Y_pressed = False
pre_key_Y_press_status = False
key_X_pressed = False
pre_key_X_press_status = False

def callback(data):
    global vx, vy, w, key_Y_pressed, key_X_pressed
    vx = float(data.axes[1] * 0.5)
    vy = float(data.axes[0] * 0.25)
    w = float(data.axes[3] * 0.4)
    if data.buttons[3] == 1:
        key_Y_pressed = True
    else:
        key_Y_pressed = False
    if data.buttons[2] == 1:
        key_X_pressed = True
    else:
        key_X_pressed = False

def main(args=None):
    global vx, vy, w, key_Y_pressed, pre_key_Y_press_status, key_X_pressed, pre_key_X_press_status
    rclpy.init(args=args)
    node = rclpy.create_node('joy11')
    
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    hw_switch_publisher = node.create_publisher(Bool, '/hwswitch', 1)
    gait_str_publisher = node.create_publisher(String, '/desired_gait_str', 1)
    node.create_subscription(Joy, 'joy', callback, 10)

    gait_str = "trot"
    hw_switch_bool = False

    def ros_publish():
        nonlocal gait_str, hw_switch_bool
        global key_Y_pressed, pre_key_Y_press_status, key_X_pressed, pre_key_X_press_status

        while rclpy.ok():
            try:
                twist_msg = Twist()
                twist_msg.linear.x = float(vx)
                twist_msg.linear.y = float(vy)
                twist_msg.angular.z = float(w)
                publisher.publish(twist_msg)

                if key_Y_pressed and not pre_key_Y_press_status:
                    hw_switch_bool = not hw_switch_bool
                    print(f'\n Switch the output status to {hw_switch_bool}')
                pre_key_Y_press_status = key_Y_pressed

                hw_switch_msg = Bool()
                hw_switch_msg.data = hw_switch_bool
                hw_switch_publisher.publish(hw_switch_msg)

                if key_X_pressed and not pre_key_X_press_status:
                    if gait_str == "trot":
                        gait_str = "walk"
                    elif gait_str == "walk":
                        gait_str = "trot"
                    print(f'\n Switch the desire gait to {gait_str}')
                pre_key_X_press_status = key_X_pressed

                gait_str_msg = String()
                gait_str_msg.data = gait_str
                gait_str_publisher.publish(gait_str_msg)

                time.sleep(1.0 / 150.0)
            except Exception:
                break

    thread = threading.Thread(target=ros_publish)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
