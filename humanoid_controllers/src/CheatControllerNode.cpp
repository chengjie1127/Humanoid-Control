//
// Created by pocket on 24-2-11.
//

#include "humanoid_controllers/humanoidController.h"
#include <atomic>
#include <rclcpp/qos.hpp>
#include <thread>

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

std::atomic_bool pause_flag{true};

void pauseCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg){
    pause_flag.store(msg->data, std::memory_order_relaxed);
}

int main(int argc, char** argv){
    rclcpp::Duration elapsedTime_(0, 0);
    // Control loop runs at a fixed target rate. Using measured wall-time span can
    // make dt much larger than intended under load, which advances the internal
    // MPC/MRT time too quickly ("requested currentTime > received plan") and can
    // destabilize the controller.
    // Keep dt consistent with the sleep rate instead.
    constexpr double kControlPeriodSec = 1.0 / 500.0;

    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("cheat_controller_node");

    //create a subscriber to pauseFlag
    auto pause_sub = nh->create_subscription<std_msgs::msg::Bool>("pauseFlag", 1, pauseCallback);
    auto controller_ready_pub = nh->create_publisher<std_msgs::msg::Bool>(
        "controllerReady", rclcpp::QoS(1).transient_local().reliable());
    std_msgs::msg::Bool controller_ready_msg;
    controller_ready_msg.data = false;
    controller_ready_pub->publish(controller_ready_msg);
    humanoid_controller::humanoidCheaterController controller;
    if (!controller.init(nh)) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to initialize the humanoid controller!");
        return -1;
    }

    // Start spinning before controller.starting() so subscriptions are serviced
    std::thread spin_thread([&](){
        rclcpp::spin(nh);
    });

    // Keep a controller-local monotonic timebase. Using /clock directly here can
    // deadlock at startup when simulation is intentionally paused and /clock stays 0.
    rclcpp::Time controller_time_ros(0, 0, RCL_ROS_TIME);
    try {
        controller.starting(controller_time_ros);
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
        const auto currentTime = Clock::now();
        const Duration desiredDuration(kControlPeriodSec);

        // Always run the controller at a fixed rate.
        // Even if the MuJoCo physics is paused, sensors are still published, and
        // keeping the controller/MPC time aligned prevents "currentTime > plan"
        // drift and large transients when the sim unpauses.
        elapsedTime_ = rclcpp::Duration::from_seconds(kControlPeriodSec);
        lastTime = currentTime;

        try {
            controller_time_ros = controller_time_ros + elapsedTime_;
            controller.update(controller_time_ros, elapsedTime_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(nh->get_logger(), "Controller update exception: %s", e.what());
        }

        const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
        std::this_thread::sleep_until(sleepTill);
    }



    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    return 0;
}