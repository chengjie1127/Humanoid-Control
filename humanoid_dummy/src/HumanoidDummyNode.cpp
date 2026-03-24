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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <humanoid_interface/HumanoidInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_msgs/srv/reset.hpp>

#include "humanoid_dummy/visualization/HumanoidVisualizer.h"

using namespace ocs2;
using namespace humanoid;

int main(int argc, char** argv) {
  using namespace std::chrono_literals;
  const std::string robotName = "humanoid";

  // Initialize ros node
  rclcpp::init(argc, argv);
  auto nodeHandle = rclcpp::Node::make_shared(robotName + "_mrt");
  // Get node parameters
  std::string taskFile, urdfFile, referenceFile;
  nodeHandle->declare_parameter<std::string>("taskFile", "");
  nodeHandle->get_parameter("taskFile", taskFile);
  nodeHandle->declare_parameter<std::string>("urdfFile", "");
  nodeHandle->get_parameter("urdfFile", urdfFile);
  nodeHandle->declare_parameter<std::string>("referenceFile", "");
  nodeHandle->get_parameter("referenceFile", referenceFile);

  // Robot interface
  HumanoidInterface interface(taskFile, urdfFile, referenceFile);

  // Initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  initObservation.mode = ModeNumber::STANCE;

  // Initial command
  TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state}, {initObservation.input});

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Reset MPC after the MRT ROS interfaces are created but before entering the dummy loop.
  mrt.resetMpcNode(initTargetTrajectories);

  // Some OCS2 setups still require a successful call to the reset service before the MPC starts processing
  // observations (otherwise it may keep warning "MPC should be reset first"). Make this bringup robust by
  // explicitly calling the reset service here.
  {
    const std::string resetServiceName = "/" + robotName + "_mpc_reset";
    auto resetClient = nodeHandle->create_client<ocs2_msgs::srv::Reset>(resetServiceName);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nodeHandle);

    RCLCPP_INFO(nodeHandle->get_logger(), "Waiting for MPC reset service: %s", resetServiceName.c_str());
    while (rclcpp::ok() && !resetClient->wait_for_service(1s)) {
      RCLCPP_WARN_THROTTLE(nodeHandle->get_logger(), *nodeHandle->get_clock(), 5000,
                           "Still waiting for MPC reset service: %s", resetServiceName.c_str());
    }

    if (rclcpp::ok()) {
      auto request = std::make_shared<ocs2_msgs::srv::Reset::Request>();
      request->reset = true;
      request->target_trajectories.time_trajectory = {0.0};

      ocs2_msgs::msg::MpcState stateMsg;
      stateMsg.value.resize(static_cast<size_t>(initObservation.state.size()));
      for (size_t i = 0; i < stateMsg.value.size(); ++i) {
        stateMsg.value[i] = static_cast<float>(initObservation.state(static_cast<vector_t::Index>(i)));
      }

      ocs2_msgs::msg::MpcInput inputMsg;
      inputMsg.value.resize(static_cast<size_t>(initObservation.input.size()));
      for (size_t i = 0; i < inputMsg.value.size(); ++i) {
        inputMsg.value[i] = static_cast<float>(initObservation.input(static_cast<vector_t::Index>(i)));
      }

      request->target_trajectories.state_trajectory = {stateMsg};
      request->target_trajectories.input_trajectory = {inputMsg};

      auto future = resetClient->async_send_request(request);
      const auto result = executor.spin_until_future_complete(future, 30s);

      if (result == rclcpp::FutureReturnCode::SUCCESS && future.get()->done) {
        RCLCPP_INFO(nodeHandle->get_logger(), "MPC reset service succeeded (%s)", resetServiceName.c_str());
      } else {
        RCLCPP_WARN(nodeHandle->get_logger(), "MPC reset service did not succeed (%s)", resetServiceName.c_str());
      }
    }
  }

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                       interface.modelSettings().contactNames3DoF);
  auto humanoidVisualizer = std::make_shared<HumanoidVisualizer>(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle);

  // Dummy legged robot
  MRT_ROS_Dummy_Loop HumanoidDummySimulator(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                                               interface.mpcSettings().mpcDesiredFrequency_);
  HumanoidDummySimulator.subscribeObservers({humanoidVisualizer});

  // run dummy
  HumanoidDummySimulator.run(initObservation, initTargetTrajectories);

  // Successful exit
  rclcpp::shutdown();
  return 0;
}
