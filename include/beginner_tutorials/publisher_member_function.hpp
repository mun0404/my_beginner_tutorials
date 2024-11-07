/**
 * @file publisher_member_function.hpp
 * @author Mohammed Munawwar (mmunawwa@umd.edu)
 * @version 0.1
 * @date 2024-11-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
 public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
          500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

 private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
