#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import ModeSchedule
from pynput import keyboard
import threading
import time


FORWARD_SPEED = 0.2
LATERAL_SPEED = 0.1
YAW_SPEED = 0.2

# Gait definitions: switchingTimes + modeSequence
# STANCE=3, LCONTACT=1, RCONTACT=2
GAITS = {
    'stance':    ModeSchedule(event_times=[0.0, 1000.0], mode_sequence=[3]),
    'pace':      ModeSchedule(event_times=[0.0, 0.50, 0.60, 1.10, 1.20], mode_sequence=[3, 1, 3, 2]),
    'slow_walk': ModeSchedule(event_times=[0.0, 0.20, 0.65, 0.85, 1.30, 1.50], mode_sequence=[3, 1, 3, 2, 3]),
    'walk':      ModeSchedule(event_times=[0.0, 0.12, 0.52, 0.64, 1.04], mode_sequence=[3, 1, 3, 2]),
    'trot':      ModeSchedule(event_times=[0.0, 0.45, 0.9], mode_sequence=[1, 2]),
}
DEFAULT_MOVE_GAIT = 'pace'  # conservative gait for in-place stepping


class KeyboardController:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 1)
        self.gait_publisher = node.create_publisher(ModeSchedule, '/humanoid_mpc_mode_schedule', 1)
        self.twist_msg = Twist()
        self.last_published_was_zero = False
        self.last_logged_command = None
        self.current_gait = 'stance'  # track current gait
        self._last_gait_publish_time = 0.0

    def publish_gait(self, gait_name, *, log: bool = True):
        if gait_name in GAITS:
            self.gait_publisher.publish(GAITS[gait_name])
            self.current_gait = gait_name
            if log:
                self.node.get_logger().info(f'Gait -> {gait_name}')
        else:
            self.node.get_logger().warn(f'Unknown gait: {gait_name}')
            return

    def is_zero_command(self):
        return (
            self.twist_msg.linear.x == 0.0 and
            self.twist_msg.linear.y == 0.0 and
            self.twist_msg.angular.z == 0.0
        )

    def command_tuple(self):
        return (
            self.twist_msg.linear.x,
            self.twist_msg.linear.y,
            self.twist_msg.angular.z,
        )

    def log_command_if_changed(self):
        current_command = self.command_tuple()
        if current_command == self.last_logged_command:
            return

        self.last_logged_command = current_command
        if self.is_zero_command():
            self.node.get_logger().info('cmd_vel -> stop')
            return

        self.node.get_logger().info(
            'cmd_vel -> x={:.2f} m/s, y={:.2f} m/s, yaw={:.2f} rad/s'.format(*current_command)
        )

    def on_press(self, key):
        try:
            c = key.char
            # Gait switching: 0=stance, 1=pace, 2=walk, 3=trot
            if c == '0':
                self.publish_gait('stance')
                return
            elif c == '1':
                self.publish_gait('pace')
                return
            elif c == '2':
                self.publish_gait('walk')
                return
            elif c == '3':
                self.publish_gait('trot')
                return
            elif c == 'z':
                # Shutdown signal can be handled gracefully
                return

            # Auto-switch to pace gait on first movement key
            if c in ('w', 's', 'a', 'd', 'q', 'e') and self.current_gait == 'stance':
                self.publish_gait(DEFAULT_MOVE_GAIT)

            if c == 'w':
                self.twist_msg.linear.x = FORWARD_SPEED
            elif c == 's':
                self.twist_msg.linear.x = -FORWARD_SPEED
            else:
                self.twist_msg.linear.x = 0.0

            if c == 'a':
                self.twist_msg.linear.y = LATERAL_SPEED
            elif c == 'd':
                self.twist_msg.linear.y = -LATERAL_SPEED
            else:
                self.twist_msg.linear.y = 0.0

            if c == 'q':
                self.twist_msg.angular.z = YAW_SPEED
            elif c == 'e':
                self.twist_msg.angular.z = -YAW_SPEED
            else:
                self.twist_msg.angular.z = 0.0
        except AttributeError:
            pass

    def on_release(self, key):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.angular.z = 0.0
        # If we were walking/stepping, revert gait to stance when the user releases keys.
        if self.current_gait != 'stance':
            self.publish_gait('stance')

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("keyboard_control")
    node.get_logger().info(
        f"Keyboard teleop ready: w/s={FORWARD_SPEED:.2f} m/s, a/d={LATERAL_SPEED:.2f} m/s, q/e={YAW_SPEED:.2f} rad/s"
    )
    node.get_logger().info("Gait keys: 0=stance, 1=pace, 2=walk, 3=trot (auto-switches to pace on first move key)")

    controller = KeyboardController(node)
    # Ensure the controller starts in stance even if teleop was previously used to command a gait.
    # (ModeSchedule QoS is volatile, so we publish once at startup and also revert on stop.)
    controller.publish_gait('stance')

    def ros_publish():
        while rclpy.ok():
            try:
                # Periodically republish gait so the controller reliably receives it (QoS is volatile).
                now = time.time()
                if (now - controller._last_gait_publish_time) >= 1.0:
                    controller.publish_gait(controller.current_gait, log=False)
                    controller._last_gait_publish_time = now

                is_zero_command = controller.is_zero_command()
                # When the command returns to zero, automatically revert gait to stance.
                if is_zero_command and not controller.last_published_was_zero and controller.current_gait != 'stance':
                    controller.publish_gait('stance')
                if (not is_zero_command) or (not controller.last_published_was_zero):
                    controller.publisher.publish(controller.twist_msg)
                    controller.last_published_was_zero = is_zero_command
                    controller.log_command_if_changed()
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