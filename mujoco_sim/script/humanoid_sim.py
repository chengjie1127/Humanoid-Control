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

init_joint_pos = np.array([0.0, 0.0, 0.37, 0.90, 0.53, 0, 0.0, 0.0, 0.37, 0.90, 0.53, 0])
init_base_pos = np.array([0, 0, 1.225])
init_base_eular_zyx = np.array([0.0, -0., 0.0])
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

    self.node.create_subscription(Float32MultiArray, "/targetTorque", self.targetTorqueCallback, 10) 
    self.node.create_subscription(Float32MultiArray, "/targetPos", self.targetPosCallback, 10) 
    self.node.create_subscription(Float32MultiArray, "/targetVel", self.targetVelCallback, 10)
    self.node.create_subscription(Float32MultiArray, "/targetKp", self.targetKpCallback, 10)
    self.node.create_subscription(Float32MultiArray, "/targetKd", self.targetKdCallback, 10)
    #set the initial joint position
    self.data.qpos[:3] = init_base_pos
    # init rpy to init quaternion
    self.data.qpos[3:7] = R.from_euler('xyz', init_base_eular_zyx).as_quat()
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
    publish_time = self.data.time
    torque_publish_time = self.data.time
    sim_epoch_start = time.time()
    
    while not glfw.window_should_close(self.window):
      # Paced by glfw.swap_buffers vsync blocking at end of loop
      
      # Run physics steps until we catch up to 1/60th of a second (if not paused)
      if not self.pause_flag:
        simstart = self.data.time
        while (self.data.time - simstart < 1.0/60.0):
          if (time.time() - sim_epoch_start >= 1.0 / self.sim_rate):
            # MIT control
            self.data.ctrl[:] = self.targetTorque + self.targetKp * (self.targetPos - self.data.qpos[-12:]) + self.targetKd * (self.targetVel - self.data.qvel[-12:])
            # Step simulation environment
            mj.mj_step(self.model, self.data)
            sim_epoch_start = time.time()
      else:
        # Give Python threads breathing room during pauses
        time.sleep(0.001)

      # Process normal physics-controlled publications if spinning, 
      # or force-publish static state if paused (self.data.time doesn't increase)
      if (not self.pause_flag and self.data.time - publish_time >= 1.0 / 500.0) or self.pause_flag:
        # * Publish joint positions and velocities
        jointsPosVel = Float32MultiArray()
        # get last 12 element of qpos and qvel
        qp = self.data.qpos[-12:].copy()
        
        if self.pause_flag:
            qv = np.zeros(12)
        else:
            qv = self.data.qvel[-12:].copy()
            
        jointsPosVel.data = np.concatenate((qp,qv)).tolist()

        self.pubJoints.publish(jointsPosVel)
        
        # * Publish body pose
        bodyOdom = Odometry()
        pos = self.data.sensor('BodyPos').data.copy()

        #add imu bias
        ori = self.data.sensor('BodyQuat').data.copy()
        # Ensure correct scipy XYZW quaternion mapping from MuJoCo WXYZ
        ori_scipy = np.array([ori[1], ori[2], ori[3], ori[0]])
        eul = R.from_quat(ori_scipy).as_euler('xyz')
        eul += imu_eular_bias
        ori_new = R.from_euler('xyz', eul).as_quat()

        if self.pause_flag:
            vel = np.zeros(3)
            angVel = np.zeros(3)
        else:
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
        
        if self.pause_flag:
            acc = np.array([0.0, 0.0, 9.81])
        else:
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
        bodyImu.orientation_covariance = [0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0]
        bodyImu.angular_velocity_covariance = [0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0]
        bodyImu.linear_acceleration_covariance = [0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0]
        self.pubImu.publish(bodyImu)

        if not self.pause_flag:
            publish_time = self.data.time

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
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    sim.simulate()
    
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == "__main__":
    main()
