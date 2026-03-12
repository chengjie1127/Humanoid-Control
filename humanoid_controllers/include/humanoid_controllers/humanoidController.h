//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <humanoid_dummy/visualization/HumanoidVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <humanoid_estimation/StateEstimateBase.h>
#include <humanoid_interface/HumanoidInterface.h>
#include <humanoid_wbc/WbcBase.h>

#include "humanoid_controllers/SafetyChecker.h"
#include "humanoid_controllers/visualization/humanoidSelfCollisionVisualization.h"

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <chrono>

namespace humanoid_controller{
using namespace ocs2;
using namespace humanoid;

class humanoidController {
 public:
  humanoidController() = default;
  virtual ~humanoidController();
  bool init(std::shared_ptr<rclcpp::Node> controller_nh);
  void update(const rclcpp::Time& time, const rclcpp::Duration& period);
  void starting(const rclcpp::Time& time);
  void stopping(const rclcpp::Time& /*time*/) { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  virtual void setupHumanoidInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  void jointStateCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
  void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void publishStandCommand();
  void publishStandCommand(const vector_t& torqueFeedforward);
  void publishDiagnosticStatus(bool fallbackMode, bool hasTorque = false, scalar_t torque0 = 0.0, scalar_t torque1 = 0.0,
                               scalar_t torque2 = 0.0);

  // Interface
  std::shared_ptr<HumanoidInterface> HumanoidInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<HumanoidVisualizer> robotVisualizer_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher_;

  //Controller Interface
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetTorquePub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetPosPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetVelPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetKpPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetKdPub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr jointPosVelSub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;

  // Node Handle
  std::shared_ptr<rclcpp::Node> controllerNh_;


 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  std::atomic_bool receivedJointState_{false}, receivedImu_{false};
  std::chrono::steady_clock::time_point lastJointStateWallTime_{};
  std::chrono::steady_clock::time_point lastImuWallTime_{};
  std::chrono::steady_clock::time_point lastDiagnosticWallTime_{};
  bool controlBlendStartInitialized_ = false;
  bool initialStanceHoldActive_ = true;
  scalar_t startupCommandBlendDuration_ = 3.0;
  scalar_t controlBlendElapsedTime_ = 0.0;
  // Walking PD blend: (disabled — using fixed PD gains like ROS1)
  bool walkingBlendActive_ = false;
  int walkingBlendIterations_ = 0;
  // Walk torque filter: (disabled — direct WBC pass-through like ROS1)
  bool walkTorqueFilterInitialized_ = false;
  vector_t filteredWalkTorque_;
  // MPC warm-up: run MPC loop on real sensor data for N seconds before commanding WBC torques
  bool mpcWarmupActive_ = true;
  int mpcWarmupIterations_ = 0;
  static constexpr int kMpcWarmupMinIterations = 1500;  // ~3s at 500Hz update rate
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
  size_t jointNum_ = 12;
  vector_t jointPos_, jointVel_;
  Eigen::Quaternion<scalar_t> quat_;
  contact_flag_t contactFlag_;
  vector3_t angularVel_, linearAccel_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;
  size_t plannedMode_ = 3;
  vector_t defalutJointPos_;
};

class humanoidCheaterController : public humanoidController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace humanoid_controller

