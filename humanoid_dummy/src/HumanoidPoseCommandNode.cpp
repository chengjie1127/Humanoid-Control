/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <string>

#include <atomic>
#include <mutex>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>

using namespace ocs2;

namespace {
scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t comHeight;
vector_t defaultJointState(12);
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity;
  return std::max(rotationTime, displacementTime);
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    // base p_x, p_y are relative to current state
    target(0) = currentPose(0) + commadLineTarget(0);
    target(1) = currentPose(1) + commadLineTarget(1);
    // base z relative to the default height
    target(2) = comHeight + commadLineTarget(2);
    // theta_z relative to current
    target(3) = currentPose(3) + commadLineTarget(3) * M_PI / 180.0;
    // theta_y, theta_x
    target(4) = currentPose(4);
    target(5) = currentPose(5);
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);

  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  const std::string robotName = "humanoid";

  // Initialize ros node
  rclcpp::init(argc, argv);
  auto nodeHandle = rclcpp::Node::make_shared(robotName + "_target");
  // Get node parameters
  std::string referenceFile;
  nodeHandle->declare_parameter<std::string>("referenceFile", "");
  nodeHandle->get_parameter("referenceFile", referenceFile);

  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity);

  const bool stdinIsTty = ::isatty(STDIN_FILENO);

  if (stdinIsTty) {
    // Interactive mode (original behavior)
    // goalPose: [deltaX, deltaY, deltaZ, deltaYaw]
    const scalar_array_t relativeBaseLimit{10.0, 10.0, 0.2, 360.0};
    TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeBaseLimit, &commandLineToTargetTrajectories);

    const std::string commandMsg = "Enter XYZ and Yaw (deg) displacements for the TORSO, separated by spaces";
    targetPoseCommand.publishKeyboardCommand(commandMsg);

    // Successful exit
    rclcpp::shutdown();
    return 0;
  }

  // Headless mode: publish an initial standing target once an observation is available.
  // This breaks the MPC/MRT startup deadlock in non-interactive environments.
  nodeHandle->declare_parameter<bool>("autoPublishInitialTarget", true);
  bool autoPublishInitialTarget = true;
  nodeHandle->get_parameter("autoPublishInitialTarget", autoPublishInitialTarget);

  std::mutex latestObservationMutex;
  SystemObservation latestObservation;
  std::atomic<bool> haveObservation{false};
  std::atomic<bool> publishedInitialTarget{false};

  TargetTrajectoriesRosPublisher targetPublisher(nodeHandle, robotName);

  auto observationCallback = [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex);
    latestObservation = ros_msg_conversions::readObservationMsg(*msg);
    haveObservation.store(true, std::memory_order_relaxed);
  };
  auto observationSub = nodeHandle->create_subscription<ocs2_msgs::msg::MpcObservation>(
      robotName + "_mpc_observation", rclcpp::QoS(1), observationCallback);

  auto timer = nodeHandle->create_wall_timer(std::chrono::milliseconds(100), [&]() {
    if (!autoPublishInitialTarget) {
      return;
    }
    if (publishedInitialTarget.load(std::memory_order_relaxed)) {
      return;
    }
    if (!haveObservation.load(std::memory_order_relaxed)) {
      return;
    }

    SystemObservation obsCopy;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex);
      obsCopy = latestObservation;
    }

    const vector_t zeroCmd = vector_t::Zero(4);  // [dX, dY, dZ, dYawDeg]
    const auto trajectories = commandLineToTargetTrajectories(zeroCmd, obsCopy);
    targetPublisher.publishTargetTrajectories(trajectories);
    publishedInitialTarget.store(true, std::memory_order_relaxed);
    RCLCPP_INFO(nodeHandle->get_logger(), "Published initial target trajectories (headless mode).\n");
  });

  RCLCPP_INFO(nodeHandle->get_logger(),
              "No TTY detected; running in headless mode. Waiting for %s and auto-publishing an initial %s if enabled.",
              (robotName + "_mpc_observation").c_str(), (robotName + "_mpc_target").c_str());

  rclcpp::spin(nodeHandle);
  rclcpp::shutdown();
  return 0;
}
