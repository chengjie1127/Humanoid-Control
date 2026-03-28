#!/usr/bin/env python3
import mujoco as mj
import numpy as np
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray,Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
from scipy.spatial.transform import Rotation as R

# init_joint_pos = np.array([0.0, 0.0, 0.37, 0.90, 0.53, 0.0, 0.0, 0.0, 0.37, 0.90, 0.53, 0.0])
init_joint_pos = np.array([-0.30, 0.0, 0.0, 0.70, -0.40, 0.0, -0.30, 0.0, 0.0, 0.70, -0.40, 0.0])
# init_base_pos = np.array([0.0, 0.0, 1.225])
init_base_pos = np.array([0.0, 0.0, 0.759])
init_base_eular_zyx = np.array([0.0, -0.0, 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])

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
    self.targetKp = np.zeros(12)
    self.targetKd = np.zeros(12)

    self.pubJoints = self.node.create_publisher(Float32MultiArray, '/jointsPosVel', 10)
    self.pubOdom = self.node.create_publisher(Odometry, '/ground_truth/state', 10)
    self.pubImu = self.node.create_publisher(Imu, '/imu', 10)
    self.pubRealTorque = self.node.create_publisher(Float32MultiArray, '/realTorque', 10)

    self.pubFootContact = self.node.create_publisher(Float32MultiArray, '/foot_contact_flags', 10)
    
    # 【新增】：提前获取地面和左右脚连杆的 ID
    self._ground_geom_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, 'ground')
    self._left_foot_body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, 'left_ankle_roll_link')
    self._right_foot_body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, 'right_ankle_roll_link')

    self.node.create_subscription(Float32MultiArray, "/targetTorque", self.targetTorqueCallback, 10) 
    self.node.create_subscription(Float32MultiArray, "/targetPos", self.targetPosCallback, 10) 
    self.node.create_subscription(Float32MultiArray, "/targetVel", self.targetVelCallback, 10)
    self.node.create_subscription(Float32MultiArray, "/targetKp", self.targetKpCallback, 10)
    self.node.create_subscription(Float32MultiArray, "/targetKd", self.targetKdCallback, 10)
    #set the initial joint position
    self.data.qpos[:3] = init_base_pos
    # init rpy to init quaternion
    scipy_quat = R.from_euler('xyz', init_base_eular_zyx).as_quat()
    self.data.qpos[3:7] = [scipy_quat[3], scipy_quat[0], scipy_quat[1], scipy_quat[2]]
    self.data.qpos[-12:] = init_joint_pos

    self.data.qvel[:3] = np.array([0, 0, 0])
    self.data.qvel[-12:] = np.zeros(12)

    # * show the model
    mj.mj_step(self.model, self.data)
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
    publish_time = self.data.time - 1.0 / 500.0
    torque_publish_time = self.data.time - 1.0 / 40.0
    sim_epoch_start = time.time()
    while not glfw.window_should_close(self.window):
      simstart = self.data.time

      while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):

        if (time.time() - sim_epoch_start >= 1.0 / self.sim_rate):
          # MIT control
          self.data.ctrl[:] = self.targetTorque + self.targetKp * (self.targetPos - self.data.qpos[-12:]) + self.targetKd * (self.targetVel - self.data.qvel[-12:])
          # Step simulation environment
          mj.mj_step(self.model, self.data)
          sim_epoch_start = time.time()

        
        if (self.data.time - publish_time >= 1.0 / 500.0):
          # * Publish joint positions and velocities
          jointsPosVel = Float32MultiArray()
          # get last 12 element of qpos and qvel
          qp = self.data.qpos[-12:].copy()
          qv = self.data.qvel[-12:].copy()
          jointsPosVel.data = np.concatenate((qp,qv)).tolist()

          self.pubJoints.publish(jointsPosVel)

          # * Publish Foot Contact
          footContact = Float32MultiArray()
          footContact.data = self._compute_foot_contact_flags().tolist()
          self.pubFootContact.publish(footContact)

          # * Publish body pose
          bodyOdom = Odometry()
          # pos = self.data.sensor('BodyPos').data.copy()

          # #add imu bias
          # ori = self.data.sensor('BodyQuat').data.copy()
          # ori = R.from_quat(ori).as_euler('xyz')
          # ori += imu_eular_bias
          # ori = R.from_euler('xyz', ori).as_quat()

          # vel = self.data.qvel[:3].copy()
          # angVel = self.data.sensor('BodyGyro').data.copy()

          # bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
          # bodyOdom.pose.pose.position.x = pos[0]
          # bodyOdom.pose.pose.position.y = pos[1]
          # bodyOdom.pose.pose.position.z = pos[2]
          # bodyOdom.pose.pose.orientation.x = ori[1]
          # bodyOdom.pose.pose.orientation.y = ori[2]
          # bodyOdom.pose.pose.orientation.z = ori[3]
          # bodyOdom.pose.pose.orientation.w = ori[0]
          # bodyOdom.twist.twist.linear.x = vel[0]
          # bodyOdom.twist.twist.linear.y = vel[1]
          # bodyOdom.twist.twist.linear.z = vel[2]
          # bodyOdom.twist.twist.angular.x = angVel[0]
          # bodyOdom.twist.twist.angular.y = angVel[1]
          # bodyOdom.twist.twist.angular.z = angVel[2]
          # self.pubOdom.publish(bodyOdom)

          # bodyImu = Imu()
          # acc = self.data.sensor('BodyAcc').data.copy()
          # bodyImu.header.stamp = self.node.get_clock().now().to_msg()
          # bodyImu.angular_velocity.x = angVel[0]
          # bodyImu.angular_velocity.y = angVel[1]
          # bodyImu.angular_velocity.z = angVel[2]
          # bodyImu.linear_acceleration.x = acc[0]
          # bodyImu.linear_acceleration.y = acc[1]
          # bodyImu.linear_acceleration.z = acc[2]
          # bodyImu.orientation.x = ori[1]
          # bodyImu.orientation.y = ori[2]
          # bodyImu.orientation.z = ori[3]
          # bodyImu.orientation.w = ori[0]

          #add imu bias
          # pos = self.data.qpos[:3].copy()
          # mj_quat = self.data.qpos[3:7].copy()
          # final_quat = [mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]]

          # vel = self.data.qvel[:3].copy()
          # angVel = self.data.qvel[3:6].copy()

          # bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
          # bodyOdom.pose.pose.position.x = pos[0]
          # bodyOdom.pose.pose.position.y = pos[1]
          # bodyOdom.pose.pose.position.z = pos[2]
          # bodyOdom.pose.pose.orientation.x = final_quat[0]
          # bodyOdom.pose.pose.orientation.y = final_quat[1]
          # bodyOdom.pose.pose.orientation.z = final_quat[2]
          # bodyOdom.pose.pose.orientation.w = final_quat[3]
          # bodyOdom.twist.twist.linear.x = vel[0]
          # bodyOdom.twist.twist.linear.y = vel[1]
          # bodyOdom.twist.twist.linear.z = vel[2]
          # bodyOdom.twist.twist.angular.x = angVel[0]
          # bodyOdom.twist.twist.angular.y = angVel[1]
          # bodyOdom.twist.twist.angular.z = angVel[2]
          # self.pubOdom.publish(bodyOdom)

          # bodyImu = Imu()
          # acc = self.data.sensor('BodyAcc').data.copy()
          # bodyImu.header.stamp = self.node.get_clock().now().to_msg()
          # bodyImu.angular_velocity.x = angVel[0]
          # bodyImu.angular_velocity.y = angVel[1]
          # bodyImu.angular_velocity.z = angVel[2]
          # bodyImu.linear_acceleration.x = acc[0]
          # bodyImu.linear_acceleration.y = acc[1]
          # bodyImu.linear_acceleration.z = acc[2]
          # bodyImu.orientation.x = final_quat[0]
          # bodyImu.orientation.y = final_quat[1]
          # bodyImu.orientation.z = final_quat[2]
          # bodyImu.orientation.w = final_quat[3]
          # bodyImu.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          # bodyImu.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          # bodyImu.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          # self.pubImu.publish(bodyImu)

          # ================= 1. 上帝视角 (给 Cheat Controller) =================
          pos = self.data.qpos[:3].copy()
          mj_quat = self.data.qpos[3:7].copy()
          odom_quat = [mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]]

          vel = self.data.qvel[:3].copy()
          odom_angvel = self.data.qvel[3:6].copy()

          # ================= 2. 真实传感器视角 (给 Normal Controller) =================
          acc = self.data.sensor('BodyAcc').data.copy()
          gyro = self.data.sensor('BodyGyro').data.copy()
          sen_quat = self.data.sensor('BodyQuat').data.copy()
          
          ori_euler = R.from_quat([sen_quat[1], sen_quat[2], sen_quat[3], sen_quat[0]]).as_euler('xyz')
          ori_euler += imu_eular_bias
          imu_quat = R.from_euler('xyz', ori_euler).as_quat()

          # ================= 3. 赋值给 Odom =================
          bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
          bodyOdom.pose.pose.position.x = pos[0]
          bodyOdom.pose.pose.position.y = pos[1]
          bodyOdom.pose.pose.position.z = pos[2]
          bodyOdom.pose.pose.orientation.x = odom_quat[0]
          bodyOdom.pose.pose.orientation.y = odom_quat[1]
          bodyOdom.pose.pose.orientation.z = odom_quat[2]
          bodyOdom.pose.pose.orientation.w = odom_quat[3]
          bodyOdom.twist.twist.linear.x = vel[0]
          bodyOdom.twist.twist.linear.y = vel[1]
          bodyOdom.twist.twist.linear.z = vel[2]
          bodyOdom.twist.twist.angular.x = odom_angvel[0]
          bodyOdom.twist.twist.angular.y = odom_angvel[1]
          bodyOdom.twist.twist.angular.z = odom_angvel[2]
          self.pubOdom.publish(bodyOdom)

          # ================= 4. 赋值给 IMU =================
          bodyImu = Imu()
          bodyImu.header.stamp = self.node.get_clock().now().to_msg()
          bodyImu.angular_velocity.x = gyro[0]
          bodyImu.angular_velocity.y = gyro[1]
          bodyImu.angular_velocity.z = gyro[2]
          bodyImu.linear_acceleration.x = acc[0]
          bodyImu.linear_acceleration.y = acc[1]
          bodyImu.linear_acceleration.z = acc[2]
          bodyImu.orientation.x = imu_quat[0]
          bodyImu.orientation.y = imu_quat[1]
          bodyImu.orientation.z = imu_quat[2]
          bodyImu.orientation.w = imu_quat[3]
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
      if self.pause_flag:
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
        # pos = self.data.sensor('BodyPos').data.copy()

        # #add imu bias
        # ori = self.data.sensor('BodyQuat').data.copy()
        # ori = R.from_quat(ori).as_euler('xyz')
        # ori += imu_eular_bias
        # ori = R.from_euler('xyz', ori).as_quat()

        # vel = self.data.qvel[:3].copy()
        # angVel = self.data.sensor('BodyGyro').data.copy()
        # bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
        # bodyOdom.pose.pose.position.x = pos[0]
        # bodyOdom.pose.pose.position.y = pos[1]
        # bodyOdom.pose.pose.position.z = pos[2]
        # bodyOdom.pose.pose.orientation.x = ori[1]
        # bodyOdom.pose.pose.orientation.y = ori[2]
        # bodyOdom.pose.pose.orientation.z = ori[3]
        # bodyOdom.pose.pose.orientation.w = ori[0]
        # bodyOdom.twist.twist.linear.x = 0.0
        # bodyOdom.twist.twist.linear.y = 0.0
        # bodyOdom.twist.twist.linear.z = 0.0
        # bodyOdom.twist.twist.angular.x = 0.0
        # bodyOdom.twist.twist.angular.y = 0.0
        # bodyOdom.twist.twist.angular.z = 0.0
        # self.pubOdom.publish(bodyOdom)

        # bodyImu = Imu()
        # bodyImu.header.stamp = self.node.get_clock().now().to_msg()
        # bodyImu.angular_velocity.x = 0.0
        # bodyImu.angular_velocity.y = 0.0
        # bodyImu.angular_velocity.z = 0.0
        # bodyImu.linear_acceleration.x = 0.0
        # bodyImu.linear_acceleration.y = 0.0
        # bodyImu.linear_acceleration.z = 9.81
        # bodyImu.orientation.x = ori[1]
        # bodyImu.orientation.y = ori[2]
        # bodyImu.orientation.z = ori[3]
        # bodyImu.orientation.w = ori[0]

        #add imu bias
        # pos = self.data.qpos[:3].copy()
        # mj_quat = self.data.qpos[3:7].copy()
        # final_quat = [mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]]

        # vel = self.data.qvel[:3].copy()
        # angVel = self.data.qvel[3:6].copy()
        
        # bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
        # bodyOdom.pose.pose.position.x = pos[0]
        # bodyOdom.pose.pose.position.y = pos[1]
        # bodyOdom.pose.pose.position.z = pos[2]
        # bodyOdom.pose.pose.orientation.x = final_quat[0]
        # bodyOdom.pose.pose.orientation.y = final_quat[1]
        # bodyOdom.pose.pose.orientation.z = final_quat[2]
        # bodyOdom.pose.pose.orientation.w = final_quat[3]
        # bodyOdom.twist.twist.linear.x = 0.0
        # bodyOdom.twist.twist.linear.y = 0.0
        # bodyOdom.twist.twist.linear.z = 0.0
        # bodyOdom.twist.twist.angular.x = 0.0
        # bodyOdom.twist.twist.angular.y = 0.0
        # bodyOdom.twist.twist.angular.z = 0.0
        # self.pubOdom.publish(bodyOdom)

        # bodyImu = Imu()
        # bodyImu.header.stamp = self.node.get_clock().now().to_msg()
        # bodyImu.angular_velocity.x = 0.0
        # bodyImu.angular_velocity.y = 0.0
        # bodyImu.angular_velocity.z = 0.0
        # bodyImu.linear_acceleration.x = 0.0
        # bodyImu.linear_acceleration.y = 0.0
        # bodyImu.linear_acceleration.z = 9.81
        # bodyImu.orientation.x = final_quat[0]
        # bodyImu.orientation.y = final_quat[1]
        # bodyImu.orientation.z = final_quat[2]
        # bodyImu.orientation.w = final_quat[3]
        # self.pubImu.publish(bodyImu)

        # ================= 1. 上帝视角 (给 Cheat Controller) =================
        pos = self.data.qpos[:3].copy()
        mj_quat = self.data.qpos[3:7].copy()
        odom_quat = [mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]]

        # ================= 2. 真实传感器视角 (给 Normal Controller) =================
        sen_quat = self.data.sensor('BodyQuat').data.copy()
        ori_euler = R.from_quat([sen_quat[1], sen_quat[2], sen_quat[3], sen_quat[0]]).as_euler('xyz')
        ori_euler += imu_eular_bias
        imu_quat = R.from_euler('xyz', ori_euler).as_quat()

        # ================= 3. 赋值给 Odom =================
        bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
        bodyOdom.pose.pose.position.x = pos[0]
        bodyOdom.pose.pose.position.y = pos[1]
        bodyOdom.pose.pose.position.z = pos[2]
        bodyOdom.pose.pose.orientation.x = odom_quat[0]
        bodyOdom.pose.pose.orientation.y = odom_quat[1]
        bodyOdom.pose.pose.orientation.z = odom_quat[2]
        bodyOdom.pose.pose.orientation.w = odom_quat[3]
        bodyOdom.twist.twist.linear.x = 0.0
        bodyOdom.twist.twist.linear.y = 0.0
        bodyOdom.twist.twist.linear.z = 0.0
        bodyOdom.twist.twist.angular.x = 0.0
        bodyOdom.twist.twist.angular.y = 0.0
        bodyOdom.twist.twist.angular.z = 0.0
        self.pubOdom.publish(bodyOdom)

        # ================= 4. 赋值给 IMU =================
        bodyImu = Imu()
        bodyImu.header.stamp = self.node.get_clock().now().to_msg()
        bodyImu.angular_velocity.x = 0.0
        bodyImu.angular_velocity.y = 0.0
        bodyImu.angular_velocity.z = 0.0
        bodyImu.linear_acceleration.x = 0.0
        bodyImu.linear_acceleration.y = 0.0
        bodyImu.linear_acceleration.z = 9.81
        bodyImu.orientation.x = imu_quat[0]
        bodyImu.orientation.y = imu_quat[1]
        bodyImu.orientation.z = imu_quat[2]
        bodyImu.orientation.w = imu_quat[3]
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

  def _compute_foot_contact_flags(self):
    left_contact = 0.0
    right_contact = 0.0
    for contact_index in range(self.data.ncon):
      contact = self.data.contact[contact_index]
      geom1 = int(contact.geom1)
      geom2 = int(contact.geom2)

      if geom1 == self._ground_geom_id:
        other_geom = geom2
      elif geom2 == self._ground_geom_id:
        other_geom = geom1
      else:
        continue

      body_id = self.model.geom_bodyid[other_geom]
      if body_id == self._left_foot_body_id:
        left_contact = 1.0
      elif body_id == self._right_foot_body_id:
        right_contact = 1.0

    return np.array([left_contact, right_contact], dtype=np.float64)

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
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    sim.simulate()
    
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == "__main__":
    main()
