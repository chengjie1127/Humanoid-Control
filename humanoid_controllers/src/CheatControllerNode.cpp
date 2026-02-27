//
// Created by pocket on 24-2-11.
//

#include "humanoid_controllers/humanoidController.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

bool pause_flag = false;

void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg){
    pause_flag = msg->data;
    std::cerr << "pause_flag: " << pause_flag << std::endl;
}

int main(int argc, char** argv){
    rclcpp::Duration elapsedTime_ = rclcpp::Duration::from_seconds(0.0);

    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("humanoid_controller_node");
    humanoid_controller::HybridJointInterface* robot_hw;
    //create a subscriber to pauseFlag
    auto pause_sub = nh->create_subscription<std_msgs::msg::Bool>("pauseFlag", 1, pauseCallback);
    humanoid_controller::humanoidCheaterController controller;
    if (!controller.init(robot_hw, nh)) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to initialize the humanoid controller!");
        return -1;
    }

    auto startTime = Clock::now();
    auto startTimeROS = nh->now();
    controller.starting(startTimeROS);
    auto lastTime = startTime;

    //create a thread to spin the node
    std::thread spin_thread([nh](){
        rclcpp::spin(nh);
    });
    spin_thread.detach();

    while(rclcpp::ok()){
        if (!pause_flag)
        {
            const auto currentTime = Clock::now();
            // Compute desired duration rounded to clock decimation
            const Duration desiredDuration(1.0 / 500);

            // Get change in time
            Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime);
            elapsedTime_ = rclcpp::Duration::from_seconds(time_span.count());
            lastTime = currentTime;

            // Check cycle time for excess delay
//            const double cycle_time_error = (elapsedTime_ - rclcpp::Duration::from_seconds(desiredDuration.count())).toSec();
//            if (cycle_time_error > cycleTimeErrorThreshold_) {
//                ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
//                                                                           << "cycle time: " << elapsedTime_ << "s, "
//                                                                           << "threshold: " << cycleTimeErrorThreshold_ << "s");
//            }

            // Control
            // let the controller compute the new command (via the controller manager)
            controller.update(nh->now(), elapsedTime_);

            // Sleep
            const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
            std::this_thread::sleep_until(sleepTill);
        }
    }



    rclcpp::shutdown();
    return 0;
}