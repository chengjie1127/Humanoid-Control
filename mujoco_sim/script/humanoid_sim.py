#!/usr/bin/env python3
import time

import mujoco as mj
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from mujoco.glfw import glfw
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from mujoco_base import MuJoCoBase


init_joint_pos = np.array([0.0, 0.0, 0.37, 0.90, 0.53, 0, 0.0, 0.0, 0.37, 0.90, 0.53, 0])
init_base_pos = np.array([0, 0, 1.225])
init_base_eular_zyx = np.array([0.0, -0.0, 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])


class HumanoidSim(MuJoCoBase):
    def __init__(self, xml_path, node):
        super().__init__(xml_path, node)
        self.simend = 1000.0
        self.sim_rate = 1000.0

        self.targetPos = init_joint_pos
        self.targetVel = np.zeros(12)
        self.targetTorque = np.zeros(12)
        self.targetKp = np.zeros(12)
        self.targetKd = np.zeros(12)

        self.pubJoints = self.node.create_publisher(Float32MultiArray, '/jointsPosVel', 10)
        self.pubOdom = self.node.create_publisher(Odometry, '/ground_truth/state', 10)
        self.pubImu = self.node.create_publisher(Imu, '/imu', 10)
        self.pubRealTorque = self.node.create_publisher(Float32MultiArray, '/realTorque', 10)

        self.node.create_subscription(Float32MultiArray, '/targetTorque', self.targetTorqueCallback, 10)
        self.node.create_subscription(Float32MultiArray, '/targetPos', self.targetPosCallback, 10)
        self.node.create_subscription(Float32MultiArray, '/targetVel', self.targetVelCallback, 10)
        self.node.create_subscription(Float32MultiArray, '/targetKp', self.targetKpCallback, 10)
        self.node.create_subscription(Float32MultiArray, '/targetKd', self.targetKdCallback, 10)

        self.data.qpos[:3] = init_base_pos
        self.data.qpos[3:7] = R.from_euler('xyz', init_base_eular_zyx).as_quat()
        self.data.qpos[-12:] = init_joint_pos

        self.data.qvel[:3] = np.array([0, 0, 0])
        self.data.qvel[-12:] = np.zeros(12)

        mj.mj_step(self.model, self.data)
        self.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True

        viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        mj.mjv_updateScene(
            self.model,
            self.data,
            self.opt,
            None,
            self.cam,
            mj.mjtCatBit.mjCAT_ALL.value,
            self.scene,
        )
        mj.mjr_render(viewport, self.scene, self.context)

    def targetTorqueCallback(self, data):
        self.targetTorque = data.data

    def targetPosCallback(self, data):
        self.targetPos = data.data

    def targetVelCallback(self, data):
        self.targetVel = data.data

    def targetKpCallback(self, data):
        self.targetKp = data.data

    def targetKdCallback(self, data):
        self.targetKd = data.data

    def reset(self):
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

    def _now_msg(self):
        return self.node.get_clock().now().to_msg()

    def _publish_state(self, paused=False):
        jointsPosVel = Float32MultiArray()
        qp = self.data.qpos[-12:].copy()
        qv = np.zeros(12) if paused else self.data.qvel[-12:].copy()
        jointsPosVel.data = np.concatenate((qp, qv)).tolist()
        self.pubJoints.publish(jointsPosVel)

        bodyOdom = Odometry()
        pos = self.data.sensor('BodyPos').data.copy()

        ori = self.data.sensor('BodyQuat').data.copy()
        ori = R.from_quat(ori).as_euler('xyz')
        ori += imu_eular_bias
        ori = R.from_euler('xyz', ori).as_quat()

        vel = np.zeros(3) if paused else self.data.qvel[:3].copy()
        angVel = np.zeros(3) if paused else self.data.sensor('BodyGyro').data.copy()

        bodyOdom.header.stamp = self._now_msg()
        bodyOdom.pose.pose.position.x = float(pos[0])
        bodyOdom.pose.pose.position.y = float(pos[1])
        bodyOdom.pose.pose.position.z = float(pos[2])
        bodyOdom.pose.pose.orientation.x = float(ori[1])
        bodyOdom.pose.pose.orientation.y = float(ori[2])
        bodyOdom.pose.pose.orientation.z = float(ori[3])
        bodyOdom.pose.pose.orientation.w = float(ori[0])
        bodyOdom.twist.twist.linear.x = float(vel[0])
        bodyOdom.twist.twist.linear.y = float(vel[1])
        bodyOdom.twist.twist.linear.z = float(vel[2])
        bodyOdom.twist.twist.angular.x = float(angVel[0])
        bodyOdom.twist.twist.angular.y = float(angVel[1])
        bodyOdom.twist.twist.angular.z = float(angVel[2])
        self.pubOdom.publish(bodyOdom)

        bodyImu = Imu()
        acc = np.array([0.0, 0.0, 9.81]) if paused else self.data.sensor('BodyAcc').data.copy()
        bodyImu.header.stamp = self._now_msg()
        bodyImu.angular_velocity.x = float(angVel[0])
        bodyImu.angular_velocity.y = float(angVel[1])
        bodyImu.angular_velocity.z = float(angVel[2])
        bodyImu.linear_acceleration.x = float(acc[0])
        bodyImu.linear_acceleration.y = float(acc[1])
        bodyImu.linear_acceleration.z = float(acc[2])
        bodyImu.orientation.x = float(ori[1])
        bodyImu.orientation.y = float(ori[2])
        bodyImu.orientation.z = float(ori[3])
        bodyImu.orientation.w = float(ori[0])
        bodyImu.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        bodyImu.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        bodyImu.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pubImu.publish(bodyImu)

    def simulate(self):
        publish_time = self.data.time
        torque_publish_time = self.data.time
        sim_epoch_start = time.time()

        while not glfw.window_should_close(self.window) and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            simstart = self.data.time

            while self.data.time - simstart <= 1.0 / 60.0 and not self.pause_flag:
                rclpy.spin_once(self.node, timeout_sec=0.0)

                if time.time() - sim_epoch_start >= 1.0 / self.sim_rate:
                    self.data.ctrl[:] = self.targetTorque + self.targetKp * (
                        self.targetPos - self.data.qpos[-12:]
                    ) + self.targetKd * (self.targetVel - self.data.qvel[-12:])
                    mj.mj_step(self.model, self.data)
                    sim_epoch_start = time.time()

                if self.data.time - publish_time >= 1.0 / 500.0:
                    self._publish_state(paused=False)
                    publish_time = self.data.time

            if self.data.time - torque_publish_time >= 1.0 / 40.0:
                targetTorque = Float32MultiArray()
                targetTorque.data = self.data.ctrl[:].tolist()
                self.pubRealTorque.publish(targetTorque)
                torque_publish_time = self.data.time

            if self.data.time >= self.simend:
                break

            if self.pause_flag:
                self._publish_state(paused=True)

            viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
            mj.mjv_updateScene(
                self.model,
                self.data,
                self.opt,
                None,
                self.cam,
                mj.mjtCatBit.mjCAT_ALL.value,
                self.scene,
            )
            mj.mjr_render(viewport, self.scene, self.context)
            glfw.swap_buffers(self.window)
            glfw.poll_events()

        glfw.terminate()


def main():
    rclpy.init()
    node = rclpy.create_node('humanoid_sim')

    pkg_share = get_package_share_directory('humanoid_legged_description')
    xml_path = pkg_share + '/mjcf/humanoid_legged.xml'

    sim = HumanoidSim(xml_path, node)
    sim.reset()
    sim.simulate()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
