//
// Created by pocket on 24-2-11.
//

#include "humanoid_controllers/humanoidController.h"
#include <atomic>
#include <thread>

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

std::atomic_bool pause_flag{true};

void pauseCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg){
    pause_flag.store(msg->data, std::memory_order_relaxed);
    std::cerr << "pause_flag: " << pause_flag.load(std::memory_order_relaxed) << std::endl;
}

int main(int argc, char** argv){
    rclcpp::Duration elapsedTime_(0, 0);
    constexpr double kMaxControlPeriodSec = 0.01;

    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("normal_controller_node");
    
    //create a subscriber to pauseFlag
    auto pause_sub = nh->create_subscription<std_msgs::msg::Bool>("pauseFlag", 1, pauseCallback);
    auto controller_ready_pub = nh->create_publisher<std_msgs::msg::Bool>("controllerReady", 1);
    std_msgs::msg::Bool controller_ready_msg;
    controller_ready_msg.data = false;
    controller_ready_pub->publish(controller_ready_msg);
    humanoid_controller::humanoidController controller;
    if (!controller.init(nh)) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to initialize the humanoid controller!");
        return -1;
    }

    // Start spinning before controller.starting() so subscriptions are serviced
    std::thread spin_thread([&](){
        rclcpp::spin(nh);
    });

    auto startTimeROS = nh->now();
    try {
        controller.starting(startTimeROS);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(nh->get_logger(), "Controller starting exception: %s", e.what());
        rclcpp::shutdown();
        if (spin_thread.joinable()) {
            spin_thread.join();
        }
        return -1;
    }
    controller_ready_msg.data = true;
    controller_ready_pub->publish(controller_ready_msg);
    auto lastTime = Clock::now();

    while(rclcpp::ok()){
        if (!pause_flag.load(std::memory_order_relaxed))
        {
            const auto currentTime = Clock::now();
            // Compute desired duration rounded to clock decimation
            const Duration desiredDuration(1.0 / 500);

            // Get change in time
            Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime);
            const double clampedPeriod = std::min(time_span.count(), kMaxControlPeriodSec);
            elapsedTime_ = rclcpp::Duration::from_seconds(clampedPeriod);
            lastTime = currentTime;

            // Check cycle time for excess delay
//            const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
//            if (cycle_time_error > cycleTimeErrorThreshold_) {
//                ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
//                                                                           << "cycle time: " << elapsedTime_ << "s, "
//                                                                           << "threshold: " << cycleTimeErrorThreshold_ << "s");
//            }

            // Control
            // let the controller compute the new command (via the controller manager)
            try {
                controller.update(nh->now(), elapsedTime_);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(nh->get_logger(), "Controller update exception: %s", e.what());
            }

            // Sleep
            const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
            std::this_thread::sleep_until(sleepTill);
        }
    }



    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    return 0;
}