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

#include "humanoid_dummy/gait/GaitKeyboardPublisher.h"

#include <algorithm>
#include <sstream>
#include <iostream>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/msg/mode_schedule.hpp>

#include "humanoid_dummy/gait/ModeSequenceTemplateRos.h"

namespace ocs2 {
namespace humanoid {

namespace {
std::vector<std::string> splitWords(const std::string& input) {
  std::istringstream stream(input);
  std::vector<std::string> words;
  std::string word;
  while (stream >> word) {
    words.push_back(word);
  }
  return words;
}
}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitKeyboardPublisher::GaitKeyboardPublisher(std::shared_ptr<rclcpp::Node> node, const std::string& gaitFile, const std::string& robotName,
                                             bool verbose) : node_(node) {
  RCLCPP_INFO_STREAM(node_->get_logger(), robotName + "_mpc_mode_schedule node is setting up ...");
  loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = node_->create_publisher<ocs2_msgs::msg::ModeSchedule>(robotName + "_mpc_mode_schedule", 1);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), robotName + "_mpc_mode_schedule command node is ready.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::getKeyboardCommand() {
  const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
  std::cout << commadMsg << ": " << std::flush;

  std::string commandLineRaw;
  if (!std::getline(std::cin, commandLineRaw)) {
    if (!rclcpp::ok()) {
      return;
    }

    if (std::cin.eof()) {
      std::cout << "Input stream closed. Exiting gait command terminal.\n" << std::flush;
      rclcpp::shutdown();
      return;
    }

    std::cin.clear();
    std::cout << "Input read failed.\n" << std::flush;
    return;
  }

  const auto commandLine = splitWords(commandLineRaw);

  if (commandLine.empty()) {
    return;
  }

  if (commandLine.size() > 1) {
    std::cout << "WARNING: The command should be a single word." << std::endl;
    return;
  }

  // lower case transform
  auto gaitCommand = commandLine.front();
  std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(), ::tolower);

  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }

  try {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    modeSequenceTemplatePublisher_->publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
    std::cout << "Published gait command: " << gaitCommand << "\n" << std::flush;
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

void GaitKeyboardPublisher::publishGaitCommandFromString(const std::string& gaitCommand) {
  try {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    modeSequenceTemplatePublisher_->publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::printGaitList(const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

}  // namespace humanoid
}  // end of namespace ocs2
