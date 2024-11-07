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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
    MinimalSubscriber()
    : Node("minimal_subscriber") {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

 private:
    void topic_callback(const std_msgs::msg::String & msg) const;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
