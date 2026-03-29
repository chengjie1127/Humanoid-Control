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


namespace ocs2
{
namespace humanoid
{
FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics, const rclcpp::Node::SharedPtr& nh)
  : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics, nh)
{
  sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth/state", 10, std::bind(&FromTopicStateEstimate::callback, this, std::placeholders::_1));
}

void FromTopicStateEstimate::callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  buffer_.writeFromNonRT(*msg);
  odomReceived_.store(true, std::memory_order_relaxed);
}

vector_t FromTopicStateEstimate::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  auto* odomPtr = buffer_.readFromRT();
  if (odomPtr == nullptr)
  {
    return rbdState_;
  }
  nav_msgs::msg::Odometry odom = *odomPtr;

  Eigen::Quaternion<scalar_t> quat(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                    odom.pose.pose.orientation.z);
  auto angularVelLocal = Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                                                        odom.twist.twist.angular.z);
  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
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
