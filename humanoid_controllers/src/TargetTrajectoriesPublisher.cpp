//
// Created by qiayuan on 2022/7/24.
//

#include "humanoid_controllers/TargetTrajectoriesPublisher.h"
#include "humanoid_controllers/humanoidController.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>


using namespace ocs2;
using namespace humanoid;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
constexpr scalar_t CMD_VEL_TIME_TO_TARGET_LIMIT = 0.6;
bool SWAP_CMD_VEL_XY = false;
scalar_t CMD_VEL_SCALE_X = 1.0;
scalar_t CMD_VEL_SCALE_Y = 1.0;
scalar_t CMD_VEL_SCALE_YAW = 1.0;
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelBase = cmdVel.head(3);
  if (SWAP_CMD_VEL_XY) {
    std::swap(cmdVelBase(0), cmdVelBase(1));
  }
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVelBase;

  const scalar_t timeToTarget = std::min(TIME_TO_TARGET, CMD_VEL_TIME_TO_TARGET_LIMIT);
    
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = currentPose(2);
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
      trajectories.stateTrajectory[0].head(3) = cmdVelRot;
    trajectories.stateTrajectory[1].head(3) = cmdVelRot;
    return trajectories;
}

int main(int argc, char** argv) {
  const std::string robotName = "humanoid";

  // Initialize ros node
  rclcpp::init(argc, argv);
  auto nodeHandle = rclcpp::Node::make_shared(robotName + "_target");
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle->declare_parameter<std::string>("referenceFile", "");
  nodeHandle->get_parameter("referenceFile", referenceFile);
  nodeHandle->declare_parameter<std::string>("taskFile", "");
  nodeHandle->get_parameter("taskFile", taskFile);
  nodeHandle->declare_parameter<bool>("swap_cmd_vel_xy", false);
  nodeHandle->get_parameter("swap_cmd_vel_xy", SWAP_CMD_VEL_XY);
  nodeHandle->declare_parameter<double>("cmd_vel_scale_x", 1.0);
  nodeHandle->get_parameter("cmd_vel_scale_x", CMD_VEL_SCALE_X);
  nodeHandle->declare_parameter<double>("cmd_vel_scale_y", 1.0);
  nodeHandle->get_parameter("cmd_vel_scale_y", CMD_VEL_SCALE_Y);
  nodeHandle->declare_parameter<double>("cmd_vel_scale_yaw", 1.0);
  nodeHandle->get_parameter("cmd_vel_scale_yaw", CMD_VEL_SCALE_YAW);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  RCLCPP_INFO(nodeHandle->get_logger(), "cmd_vel scaling configured: x=%.3f y=%.3f yaw=%.3f (swap_xy=%s)",
              CMD_VEL_SCALE_X, CMD_VEL_SCALE_Y, CMD_VEL_SCALE_YAW, SWAP_CMD_VEL_XY ? "true" : "false");

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);

  rclcpp::spin(nodeHandle);
  rclcpp::shutdown();
  // Successful exit
  return 0;
}
