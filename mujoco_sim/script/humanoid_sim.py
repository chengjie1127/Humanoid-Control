#!/usr/bin/env python3
import mujoco as mj
import numpy as np
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray,Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time


def euler_xyz_to_quat_xyzw(euler_xyz):
  roll, pitch, yaw = euler_xyz
  cr = np.cos(roll * 0.5)
  sr = np.sin(roll * 0.5)
  cp = np.cos(pitch * 0.5)
  sp = np.sin(pitch * 0.5)
  cy = np.cos(yaw * 0.5)
  sy = np.sin(yaw * 0.5)
  return np.array([
      sr * cp * cy + cr * sp * sy,
      cr * sp * cy - sr * cp * sy,
      cr * cp * sy + sr * sp * cy,
      cr * cp * cy - sr * sp * sy,
  ])


def quat_wxyz_to_euler_xyz(quat_wxyz):
  w, x, y, z = quat_wxyz
  sinr_cosp = 2.0 * (w * x + y * z)
  cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
  roll = np.arctan2(sinr_cosp, cosr_cosp)

  sinp = 2.0 * (w * y - z * x)
  pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

  siny_cosp = 2.0 * (w * z + x * y)
  cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
  yaw = np.arctan2(siny_cosp, cosy_cosp)
  return np.array([roll, pitch, yaw])

init_joint_pos = np.array([-0.1, 0.0, 0.0, 0.25, -0.14, 0, -0.1, 0.0, 0.0, 0.25, -0.14, 0])
init_base_pos = np.array([0, 0, 0.793])
init_base_eular_zyx = np.array([0.0, -0., 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])
default_stand_kp = np.array([150.0, 150.0, 80.0, 150.0, 50.0, 40.0, 150.0, 150.0, 80.0, 150.0, 50.0, 40.0])
default_stand_kd = np.array([2.0, 2.0, 1.2, 2.0, 1.0, 0.8, 2.0, 2.0, 1.2, 2.0, 1.0, 0.8])

class HumanoidSim(MuJoCoBase):
  def __init__(self, xml_path, node):
    super().__init__(xml_path, node)
    self.simend = 1000.0
    self.sim_rate = 1000.0
    # print('Total number of DoFs in the model:', self.model.nv)
    # print('Generalized positions:', self.data.qpos)  
    # print('Generalized velocities:', self.data.qvel)
    # print('Actuator forces:', self.data.qfrc_actuator)
    # print('Actoator controls:', self.data.ctrl)
    # mj.set_mjcb_control(self.controller)
    # * Set subscriber and publisher

    # initialize target joint position, velocity, and torque
    self.targetPos = init_joint_pos
    self.targetVel = np.zeros(12)
    self.targetTorque = np.zeros(12)
    self.targetKp = default_stand_kp.copy()
    self.targetKd = default_stand_kd.copy()

    self.pubJoints = self.node.create_publisher(Float32MultiArray, '/jointsPosVel', 10)
    self.pubOdom = self.node.create_publisher(Odometry, '/ground_truth/state', 10)
    self.pubImu = self.node.create_publisher(Imu, '/imu', 10)
    self.pubRealTorque = self.node.create_publisher(Float32MultiArray, '/realTorque', 10)

    self.node.create_subscription(Float32MultiArray, "/targetTorque", self.targetTorqueCallback, qos_profile_sensor_data) 
    self.node.create_subscription(Float32MultiArray, "/targetPos", self.targetPosCallback, qos_profile_sensor_data) 
    self.node.create_subscription(Float32MultiArray, "/targetVel", self.targetVelCallback, qos_profile_sensor_data)
    self.node.create_subscription(Float32MultiArray, "/targetKp", self.targetKpCallback, qos_profile_sensor_data)
    self.node.create_subscription(Float32MultiArray, "/targetKd", self.targetKdCallback, qos_profile_sensor_data)
    #set the initial joint position
    self.data.qpos[:3] = init_base_pos
    # MuJoCo free-joint quaternion in qpos is wxyz, while scipy returns xyzw.
    base_quat_xyzw = euler_xyz_to_quat_xyzw(init_base_eular_zyx)
    self.data.qpos[3:7] = np.array([
      base_quat_xyzw[3],
      base_quat_xyzw[0],
      base_quat_xyzw[1],
      base_quat_xyzw[2],
    ])
    self.data.qpos[-12:] = init_joint_pos

    self.data.qvel[:] = 0.0

    # Align the floating base vertically so the support feet start in contact with the ground.
    self._snap_base_to_ground()

    # Initialize kinematics and sensors without advancing physics.
    mj.mj_forward(self.model, self.data)
    # enable contact force visualization
    self.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        self.window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    # Update scene and render
    mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                        mj.mjtCatBit.mjCAT_ALL.value, self.scene)
    mj.mjr_render(viewport, self.scene, self.context)

  def _geom_vertical_extent(self, geom_index):
    geom_type = self.model.geom_type[geom_index]
    geom_size = self.model.geom_size[geom_index]
    geom_rot = self.data.geom_xmat[geom_index].reshape(3, 3)
    world_z_in_local = geom_rot[2, :]

    if geom_type == mj.mjtGeom.mjGEOM_BOX:
      return (np.abs(world_z_in_local[0]) * geom_size[0] +
              np.abs(world_z_in_local[1]) * geom_size[1] +
              np.abs(world_z_in_local[2]) * geom_size[2])

    if geom_type == mj.mjtGeom.mjGEOM_SPHERE:
      return geom_size[0]

    if geom_type == mj.mjtGeom.mjGEOM_CAPSULE or geom_type == mj.mjtGeom.mjGEOM_CYLINDER:
      radius = geom_size[0]
      half_length = geom_size[1]
      axis_alignment = np.abs(world_z_in_local[2])
      radial_alignment = np.sqrt(max(0.0, 1.0 - axis_alignment * axis_alignment))
      return radius * radial_alignment + half_length * axis_alignment

    if geom_type == mj.mjtGeom.mjGEOM_ELLIPSOID:
      return np.sqrt(np.sum((geom_size[:3] * world_z_in_local[:3]) ** 2))

    return float(np.max(geom_size[:3]))

  def _lowest_collidable_point_z(self):
    lowest_point_z = np.inf
    for geom_index in range(self.model.ngeom):
      if self.model.geom_bodyid[geom_index] == 0:
        continue
      if self.model.geom_contype[geom_index] == 0 and self.model.geom_conaffinity[geom_index] == 0:
        continue

      geom_world_pos = self.data.geom_xpos[geom_index]
      lowest_point_z = min(lowest_point_z, geom_world_pos[2] - self._geom_vertical_extent(geom_index))

    return lowest_point_z

  def _support_geom_bottoms(self):
    support_body_names = {"left_ankle_roll_link", "right_ankle_roll_link"}
    support_bottoms = []
    for geom_index in range(self.model.ngeom):
      if self.model.geom_contype[geom_index] == 0 and self.model.geom_conaffinity[geom_index] == 0:
        continue

      body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[geom_index])
      if body_name not in support_body_names:
        continue

      geom_world_pos = self.data.geom_xpos[geom_index]
      support_bottoms.append(geom_world_pos[2] - self._geom_vertical_extent(geom_index))
    return support_bottoms

  def _snap_base_to_ground(self, desired_penetration=5.0e-4):
    mj.mj_forward(self.model, self.data)
    support_bottoms = self._support_geom_bottoms()
    if support_bottoms:
      base_shift = max(support_bottoms) + desired_penetration
    else:
      lowest_point_z = self._lowest_collidable_point_z()
      if not np.isfinite(lowest_point_z):
        return
      base_shift = lowest_point_z + desired_penetration

    if not np.isfinite(base_shift):
      return

    self.data.qpos[2] -= base_shift
    self.data.qvel[:] = 0.0
    mj.mj_forward(self.model, self.data)
    support_bottoms = self._support_geom_bottoms()
    if support_bottoms:
      print(f"[HumanoidSim] Startup base_z adjusted to {self.data.qpos[2]:.6f}; support geom bottoms={support_bottoms}")
    else:
      print(f"[HumanoidSim] Startup base_z adjusted to {self.data.qpos[2]:.6f} so lowest collidable geom starts at z={self._lowest_collidable_point_z():.6f}")
    

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
    # Set camera configuration
    self.cam.azimuth = 89.608063
    self.cam.elevation = -11.588379
    self.cam.distance = 5.0
    self.cam.lookat = np.array([0.0, 0.0, 1.5])

  # def controller(self, model, data):
  #   self.data.ctrl[0] = 100
  #   pass

  def simulate(self):
    publish_time = self.data.time
    torque_publish_time = self.data.time
    sim_epoch_start = time.time()
    
    while not glfw.window_should_close(self.window):
      self.release_pending_unpause_if_ready()
      self.release_startup_pause_if_ready()
      simstart = self.data.time
      simulation_active = (not self.pause_flag) and self.can_run_simulation()

      while (self.data.time - simstart < 1.0/60.0):
        if simulation_active:
          if (time.time() - sim_epoch_start >= 1.0 / self.sim_rate):
            # MIT control
            self.data.ctrl[:] = self.targetTorque + self.targetKp * (self.targetPos - self.data.qpos[-12:]) + self.targetKd * (self.targetVel - self.data.qvel[-12:])
            # Step simulation environment
            mj.mj_step(self.model, self.data)
            sim_epoch_start = time.time()
        else:
            # Advance local time frame minimally without physics steps so the outer GUI renders and rclpy updates
            simstart -= 1.0/60.0
            time.sleep(1.0/60.0)
        
        if (self.data.time - publish_time >= 1.0 / 500.0):
          # * Publish joint positions and velocities
          jointsPosVel = Float32MultiArray()
          # get last 12 element of qpos and qvel
          qp = self.data.qpos[-12:].copy()
          qv = self.data.qvel[-12:].copy()
          jointsPosVel.data = np.concatenate((qp,qv)).tolist()

          self.pubJoints.publish(jointsPosVel)
          # * Publish body pose
          bodyOdom = Odometry()
          pos = self.data.sensor('BodyPos').data.copy()

          #add imu bias
          ori = self.data.sensor('BodyQuat').data.copy()
          # Ensure correct scipy XYZW quaternion mapping from MuJoCo WXYZ
          eul = quat_wxyz_to_euler_xyz(ori)
          eul += imu_eular_bias
          ori_new = euler_xyz_to_quat_xyzw(eul)

          vel = self.data.qvel[:3].copy()
          angVel = self.data.sensor('BodyGyro').data.copy()

          bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
          bodyOdom.pose.pose.position.x = float(pos[0])
          bodyOdom.pose.pose.position.y = float(pos[1])
          bodyOdom.pose.pose.position.z = float(pos[2])
          bodyOdom.pose.pose.orientation.x = float(ori_new[0])
          bodyOdom.pose.pose.orientation.y = float(ori_new[1])
          bodyOdom.pose.pose.orientation.z = float(ori_new[2])
          bodyOdom.pose.pose.orientation.w = float(ori_new[3])
          bodyOdom.twist.twist.linear.x = float(vel[0])
          bodyOdom.twist.twist.linear.y = float(vel[1])
          bodyOdom.twist.twist.linear.z = float(vel[2])
          bodyOdom.twist.twist.angular.x = float(angVel[0])
          bodyOdom.twist.twist.angular.y = float(angVel[1])
          bodyOdom.twist.twist.angular.z = float(angVel[2])
          self.pubOdom.publish(bodyOdom)

          bodyImu = Imu()
          acc = self.data.sensor('BodyAcc').data.copy()
          bodyImu.header.stamp = self.node.get_clock().now().to_msg()
          bodyImu.angular_velocity.x = float(angVel[0])
          bodyImu.angular_velocity.y = float(angVel[1])
          bodyImu.angular_velocity.z = float(angVel[2])
          bodyImu.linear_acceleration.x = float(acc[0])
          bodyImu.linear_acceleration.y = float(acc[1])
          bodyImu.linear_acceleration.z = float(acc[2])
          bodyImu.orientation.x = float(ori_new[0])
          bodyImu.orientation.y = float(ori_new[1])
          bodyImu.orientation.z = float(ori_new[2])
          bodyImu.orientation.w = float(ori_new[3])
          bodyImu.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          bodyImu.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          bodyImu.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          self.pubImu.publish(bodyImu)

          publish_time = self.data.time

      if (self.data.time - torque_publish_time >= 1.0 / 40.0):
        targetTorque = Float32MultiArray()
        targetTorque.data = self.data.ctrl[:].tolist()
        self.pubRealTorque.publish(targetTorque)
        torque_publish_time = self.data.time

      if self.data.time >= self.simend:
          break
      if not simulation_active:
        # publish the state even if the simulation is paused
        # * Publish joint positions and velocities
        jointsPosVel = Float32MultiArray()
        # get last 12 element of qpos and qvel
        qp = self.data.qpos[-12:].copy()
        qv = np.zeros(12)
        jointsPosVel.data = np.concatenate((qp,qv)).tolist()

        self.pubJoints.publish(jointsPosVel)
        # * Publish body pose
        bodyOdom = Odometry()
        pos = self.data.sensor('BodyPos').data.copy()

        #add imu bias
        ori = self.data.sensor('BodyQuat').data.copy()
        eul = quat_wxyz_to_euler_xyz(ori)
        eul += imu_eular_bias
        ori_new = euler_xyz_to_quat_xyzw(eul)

        vel = self.data.qvel[:3].copy()
        angVel = self.data.sensor('BodyGyro').data.copy()
        bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
        bodyOdom.pose.pose.position.x = float(pos[0])
        bodyOdom.pose.pose.position.y = float(pos[1])
        bodyOdom.pose.pose.position.z = float(pos[2])
        bodyOdom.pose.pose.orientation.x = float(ori_new[0])
        bodyOdom.pose.pose.orientation.y = float(ori_new[1])
        bodyOdom.pose.pose.orientation.z = float(ori_new[2])
        bodyOdom.pose.pose.orientation.w = float(ori_new[3])
        bodyOdom.twist.twist.linear.x = 0.0
        bodyOdom.twist.twist.linear.y = 0.0
        bodyOdom.twist.twist.linear.z = 0.0
        bodyOdom.twist.twist.angular.x = 0.0
        bodyOdom.twist.twist.angular.y = 0.0
        bodyOdom.twist.twist.angular.z = 0.0
        self.pubOdom.publish(bodyOdom)

        bodyImu = Imu()
        bodyImu.header.stamp = self.node.get_clock().now().to_msg()
        bodyImu.angular_velocity.x = 0.0
        bodyImu.angular_velocity.y = 0.0
        bodyImu.angular_velocity.z = 0.0
        bodyImu.linear_acceleration.x = 0.0
        bodyImu.linear_acceleration.y = 0.0
        bodyImu.linear_acceleration.z = 9.81
        bodyImu.orientation.x = float(ori_new[0])
        bodyImu.orientation.y = float(ori_new[1])
        bodyImu.orientation.z = float(ori_new[2])
        bodyImu.orientation.w = float(ori_new[3])
        self.pubImu.publish(bodyImu)

      # get framebuffer viewport
      viewport_width, viewport_height = glfw.get_framebuffer_size(
          self.window)
      viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

      # Update scene and render
      mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                          mj.mjtCatBit.mjCAT_ALL.value, self.scene)
      mj.mjr_render(viewport, self.scene, self.context)

      # swap OpenGL buffers (blocking call due to v-sync)
      glfw.swap_buffers(self.window)

      # process pending GUI events, call GLFW callbacks
      glfw.poll_events()

    glfw.terminate()

def main(args=None):
    # ros init
    rclpy.init(args=args)
    node = rclpy.create_node('hector_sim')

    # get xml path
    hector_desc_path = get_package_share_directory('humanoid_legged_description')
    xml_path = hector_desc_path + "/mjcf/humanoid_legged.xml"

    sim = HumanoidSim(xml_path, node)
    sim.reset()
    
    import threading
    import traceback
    import os
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        sim.simulate()
    except Exception as e:
        print(f"\n[CRITICAL ERROR] Python Main Thread crashed in humanoid_sim.py:\n{traceback.format_exc()}\n")
        os._exit(1)
    
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == "__main__":
    main()
