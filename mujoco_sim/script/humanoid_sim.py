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
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as BuiltinTime
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


def quat_wxyz_to_xyzw(quat_wxyz):
  w, x, y, z = quat_wxyz
  return np.array([x, y, z, w])

init_joint_pos = np.array([-0.30, 0.0, 0.0, 0.70, -0.40, 0, -0.30, 0.0, 0.0, 0.70, -0.40, 0])
init_base_pos = np.array([0, 0, 0.759])
init_base_eular_zyx = np.array([0.0, -0., 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])
default_stand_kp = np.array([250.0, 250.0, 120.0, 250.0, 120.0, 80.0, 250.0, 250.0, 120.0, 250.0, 120.0, 80.0])
default_stand_kd = np.array([6.0, 6.0, 2.0, 6.0, 4.0, 2.5, 6.0, 6.0, 2.0, 6.0, 4.0, 2.5])

# Canonical controller joint order (must match humanoid_interface::ModelSettings::jointNames)
CONTROLLER_JOINT_NAMES = [
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
]

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

    # Build joint/actuator index mapping so all arrays match the controller joint order.
    self._use_joint_map = False
    self._joint_names = CONTROLLER_JOINT_NAMES
    self._qpos_adr = None
    self._qvel_adr = None
    self._act_adr = None
    try:
      qpos_adr = []
      qvel_adr = []
      for joint_name in self._joint_names:
        jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
          raise RuntimeError(f"MuJoCo joint not found: {joint_name}")
        qpos_adr.append(int(self.model.jnt_qposadr[jid]))
        qvel_adr.append(int(self.model.jnt_dofadr[jid]))

      act_for_joint = {}
      for aid in range(self.model.nu):
        trnid = self.model.actuator_trnid[aid]
        jid = int(trnid[0])
        if jid < 0:
          continue
        jname = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, jid)
        if jname and jname not in act_for_joint:
          act_for_joint[jname] = aid

      act_adr = []
      for joint_name in self._joint_names:
        if joint_name not in act_for_joint:
          raise RuntimeError(f"No actuator found for joint: {joint_name}")
        act_adr.append(int(act_for_joint[joint_name]))

      self._qpos_adr = np.array(qpos_adr, dtype=np.int32)
      self._qvel_adr = np.array(qvel_adr, dtype=np.int32)
      self._act_adr = np.array(act_adr, dtype=np.int32)
      self._use_joint_map = True
      print(f"[HumanoidSim] Using joint-name mapping for commands/states (nu={self.model.nu}).")
    except Exception as e:
      self.node.get_logger().error(f"[HumanoidSim] Joint/actuator mapping failed; falling back to qpos[-12:] ordering: {e}")
      self._use_joint_map = False

    self.pubJoints = self.node.create_publisher(Float32MultiArray, '/jointsPosVel', 10)
    self.pubOdom = self.node.create_publisher(Odometry, '/ground_truth/state', 10)
    self.pubImu = self.node.create_publisher(Imu, '/imu', 10)
    self.pubRealTorque = self.node.create_publisher(Float32MultiArray, '/realTorque', 10)
    self.pubClock = self.node.create_publisher(Clock, '/clock', 10)

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
    if self._use_joint_map:
      self.data.qpos[self._qpos_adr] = init_joint_pos
    else:
      self.data.qpos[-12:] = init_joint_pos

    self.data.qvel[:] = 0.0

    # Align the floating base vertically so the support feet start in contact with the ground.
    self._snap_base_to_ground()

    # Initialize kinematics and sensors without advancing physics.
    mj.mj_forward(self.model, self.data)

    try:
      raw_quat = self.data.sensor('BodyQuat').data.copy()
      raw_pos = self.data.sensor('BodyPos').data.copy()
      print(f"[HumanoidSim] BodyQuat raw sensor (as returned by MuJoCo) = {raw_quat}")
      print(f"[HumanoidSim] BodyPos raw sensor (as returned by MuJoCo) = {raw_pos}")
    except Exception as e:
      print(f"[HumanoidSim] Warning: could not read BodyQuat/BodyPos sensors at startup: {e}")
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

  def _sim_time_msg(self):
    sec = int(self.data.time)
    nanosec = int((self.data.time - sec) * 1e9)
    if nanosec < 0:
      nanosec = 0
    if nanosec >= 1_000_000_000:
      sec += nanosec // 1_000_000_000
      nanosec = nanosec % 1_000_000_000
    return BuiltinTime(sec=sec, nanosec=nanosec)

  def _publish_clock(self):
    msg = Clock()
    msg.clock = self._sim_time_msg()
    self.pubClock.publish(msg)

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
    arr = np.asarray(data.data, dtype=np.float64)
    if arr.size != 12:
      self.node.get_logger().error(f"/targetTorque wrong size {arr.size} (expected 12)")
      return
    self.targetTorque = arr

  def targetPosCallback(self, data):
    arr = np.asarray(data.data, dtype=np.float64)
    if arr.size != 12:
      self.node.get_logger().error(f"/targetPos wrong size {arr.size} (expected 12)")
      return
    self.targetPos = arr

  def targetVelCallback(self, data):
    arr = np.asarray(data.data, dtype=np.float64)
    if arr.size != 12:
      self.node.get_logger().error(f"/targetVel wrong size {arr.size} (expected 12)")
      return
    self.targetVel = arr

  def targetKpCallback(self, data):
    arr = np.asarray(data.data, dtype=np.float64)
    if arr.size != 12:
      self.node.get_logger().error(f"/targetKp wrong size {arr.size} (expected 12)")
      return
    self.targetKp = arr

  def targetKdCallback(self, data):
    arr = np.asarray(data.data, dtype=np.float64)
    if arr.size != 12:
      self.node.get_logger().error(f"/targetKd wrong size {arr.size} (expected 12)")
      return
    self.targetKd = arr

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
            if self._use_joint_map:
              q = self.data.qpos[self._qpos_adr].copy()
              qd = self.data.qvel[self._qvel_adr].copy()
              u = self.targetTorque + self.targetKp * (self.targetPos - q) + self.targetKd * (self.targetVel - qd)
              # Apply in the controller joint order.
              self.data.ctrl[self._act_adr] = u
            else:
              self.data.ctrl[:] = self.targetTorque + self.targetKp * (self.targetPos - self.data.qpos[-12:]) + self.targetKd * (self.targetVel - self.data.qvel[-12:])
            # Step simulation environment
            mj.mj_step(self.model, self.data)
            sim_epoch_start = time.time()
            self._publish_clock()
        else:
            # Advance local time frame minimally without physics steps so the outer GUI renders and rclpy updates
            simstart -= 1.0/60.0
            time.sleep(1.0/60.0)
            self._publish_clock()
        
        if (self.data.time - publish_time >= 1.0 / 500.0):
          # * Publish joint positions and velocities
          jointsPosVel = Float32MultiArray()
          # get last 12 element of qpos and qvel
          if self._use_joint_map:
            qp = self.data.qpos[self._qpos_adr].copy()
            qv = self.data.qvel[self._qvel_adr].copy()
          else:
            qp = self.data.qpos[-12:].copy()
            qv = self.data.qvel[-12:].copy()
          jointsPosVel.data = np.concatenate((qp,qv)).tolist()

          self.pubJoints.publish(jointsPosVel)
          # * Publish body pose
          bodyOdom = Odometry()
          pos = self.data.sensor('BodyPos').data.copy()

          # MuJoCo provides quaternions as (w, x, y, z). ROS messages use (x, y, z, w).
          # Avoid Euler-angle conversions (they can introduce convention mistakes and singularities).
          ori_wxyz = self.data.sensor('BodyQuat').data.copy()
          ori_new = quat_wxyz_to_xyzw(ori_wxyz)

          vel = self.data.qvel[:3].copy()
          angVel = self.data.sensor('BodyGyro').data.copy()

          bodyOdom.header.stamp = self._sim_time_msg()
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
          bodyImu.header.stamp = self._sim_time_msg()
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
        if self._use_joint_map:
          targetTorque.data = self.data.ctrl[self._act_adr].tolist()
        else:
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
        if self._use_joint_map:
          qp = self.data.qpos[self._qpos_adr].copy()
        else:
          qp = self.data.qpos[-12:].copy()
        qv = np.zeros(12)
        jointsPosVel.data = np.concatenate((qp,qv)).tolist()

        self.pubJoints.publish(jointsPosVel)
        # * Publish body pose
        bodyOdom = Odometry()
        pos = self.data.sensor('BodyPos').data.copy()

        ori_wxyz = self.data.sensor('BodyQuat').data.copy()
        ori_new = quat_wxyz_to_xyzw(ori_wxyz)

        vel = self.data.qvel[:3].copy()
        angVel = self.data.sensor('BodyGyro').data.copy()
        bodyOdom.header.stamp = self._sim_time_msg()
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
        bodyImu.header.stamp = self._sim_time_msg()
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

        self._publish_clock()

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
