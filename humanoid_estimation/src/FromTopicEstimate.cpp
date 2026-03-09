//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_estimation/FromTopiceEstimate.h"
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <cmath>


namespace ocs2
{
namespace humanoid
{
FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics, const rclcpp::Node::SharedPtr& nh)
  : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics)
{
  const std::vector<double> sensorToBaseRpyDefault{0.0, 0.0, 0.0};
  const auto sensorToBaseRpy = nh->declare_parameter<std::vector<double>>(
      "from_topic_estimate.sensor_to_base_rpy", sensorToBaseRpyDefault);
  if (sensorToBaseRpy.size() == 3) {
    sensorToBaseQuat_ = getQuaternionFromEulerAnglesZyx(
        vector3_t(sensorToBaseRpy[0], sensorToBaseRpy[1], sensorToBaseRpy[2]));
  }
  autoZeroInitialPitch_ = nh->declare_parameter<bool>("from_topic_estimate.auto_zero_initial_pitch", true);

  sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth/state", 10, std::bind(&FromTopicStateEstimate::callback, this, std::placeholders::_1));
}

void FromTopicStateEstimate::callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  auto* odomPtr = buffer_.readFromRT();
  if (odomPtr == nullptr)
  {
    return rbdState_;
  }
  nav_msgs::msg::Odometry odom = *odomPtr;

  Eigen::Quaternion<scalar_t> quatSensor(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                         odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  quatSensor.normalize();
  Eigen::Quaternion<scalar_t> quat = quatSensor * sensorToBaseQuat_;
  quat.normalize();
  auto angularVelSensor = Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                                                         odom.twist.twist.angular.z);
  auto angularVelLocal = sensorToBaseQuat_ * angularVelSensor;
  const vector3_t rawZyx = quatToZyx(quat);
  if (autoZeroInitialPitch_ && !initialPitchOffsetInitialized_) {
    zyxOffset_(1) = rawZyx(1);
    initialPitchOffsetInitialized_ = true;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("FromTopicStateEstimate"),
                       "Startup pitch alignment raw_pitch_deg=" << (rawZyx(1) * 180.0 / M_PI)
                                                                  << " offset_pitch_deg=" << (zyxOffset_(1) * 180.0 / M_PI));
  }
  vector3_t zyx = rawZyx - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(rawZyx, angularVelLocal));
  updateAngular(zyx, angularVelGlobal);

  // updateAngular(quatToZyx(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
  //                                                     odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
  //               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
  //                                             odom.twist.twist.angular.z));
  updateLinear(
      Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
      Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  publishMsgs(odom);

  return rbdState_;
}

}  // namespace humanoid
}  // namespace ocs2