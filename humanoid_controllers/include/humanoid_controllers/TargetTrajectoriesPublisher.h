//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <humanoid_interface/gait/MotionPhaseDefinition.h>



namespace ocs2 {
namespace humanoid {

class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(std::shared_ptr<rclcpp::Node> nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        buffer_(nh->get_clock()),
        tf2_(buffer_) {
    const auto cmdVelScaleX = nh->get_parameter("cmd_vel_scale_x").as_double();
    const auto cmdVelScaleY = nh->get_parameter("cmd_vel_scale_y").as_double();
    const auto cmdVelScaleYaw = nh->get_parameter("cmd_vel_scale_yaw").as_double();
    latestCmdVel_ = vector_t::Zero(4);
    lastCmdVelMsgWallTime_ = std::chrono::steady_clock::now();

    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh->create_subscription<ocs2_msgs::msg::MpcObservation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    auto warnMissingObservation = [nh]() {
      RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000,
                           "Ignoring command because no '%s_mpc_observation' has been received yet.",
                           "humanoid");
    };

    // goal subscriber
    auto goalCallback = [this, nh, warnMissingObservation](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
      if (latestObservation_.time == 0.0) {
        warnMissingObservation();
        return;
      }
      geometry_msgs::msg::PoseStamped pose = *msg;
      try {
        pose = buffer_.transform(pose, "odom", tf2::durationFromSec(0.2));
      } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(rclcpp::get_logger("TargetTrajectoriesPublisher"), "Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      const auto& targetState = trajectories.stateTrajectory.back();
      const auto targetPose = targetState.segment<6>(6);
      {
        std::lock_guard<std::mutex> lock(commandMutex_);
        commandMode_ = CommandMode::Goal;
      }
      RCLCPP_INFO(nh->get_logger(), "Published goal target: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                  targetPose[0], targetPose[1], targetPose[2], targetPose[3]);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmdVelCallback = [this, nh, warnMissingObservation, cmdVelScaleX, cmdVelScaleY, cmdVelScaleYaw](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
      if (latestObservation_.time == 0.0) {
        warnMissingObservation();
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x * cmdVelScaleX;
      cmdVel[1] = msg->linear.y * cmdVelScaleY;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z * cmdVelScaleYaw;
      {
        std::lock_guard<std::mutex> lock(commandMutex_);
        latestCmdVel_ = cmdVel;
        lastCmdVelMsgWallTime_ = std::chrono::steady_clock::now();
        commandMode_ = CommandMode::CmdVel;
      }

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      const auto& targetState = trajectories.stateTrajectory.back();
      const auto targetPose = targetState.segment<6>(6);
      RCLCPP_INFO(nh->get_logger(),
                  "Published cmd_vel target: x=%.3f y=%.3f z=%.3f yaw=%.3f (raw vx=%.3f vy=%.3f wz=%.3f)",
                  targetPose[0], targetPose[1], targetPose[2], targetPose[3], msg->linear.x, msg->linear.y, msg->angular.z);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    goalSub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    goalPoseSub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, goalCallback);
    cmdVelSub_ = nh->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, cmdVelCallback);

    republishTimer_ = nh->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      SystemObservation observation;
      {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        observation = latestObservation_;
      }
      if (observation.time == 0.0) {
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      {
        std::lock_guard<std::mutex> lock(commandMutex_);
        if (commandMode_ == CommandMode::Goal) {
          return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (commandMode_ == CommandMode::CmdVel &&
            now - lastCmdVelMsgWallTime_ <= std::chrono::milliseconds(250)) {
          cmdVel = latestCmdVel_;
        } else {
          commandMode_ = CommandMode::Idle;
          latestCmdVel_.setZero();
        }
      }

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, observation);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    });
  }

 private:
  enum class CommandMode { Idle, CmdVel, Goal };

  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPoseSub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::TimerBase::SharedPtr republishTimer_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
  mutable std::mutex commandMutex_;
  CommandMode commandMode_ = CommandMode::Idle;
  vector_t latestCmdVel_;
  std::chrono::steady_clock::time_point lastCmdVelMsgWallTime_;
};

}  // namespace humanoid
}  // namespace ocs2
