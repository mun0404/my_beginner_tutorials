/**
 * @file subscriber_member_function.hpp
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

/**
 * @class BasicSubscriber
 * @brief A basic subscriber node class that listens to messages on 
 * a specified topic.
 */
class BasicSubscriber : public rclcpp::Node {
 public:
    /**
     * @brief Constructs a new BasicSubscriber object and initializes the
     * subscription to the "topic" topic.
     */
    BasicSubscriber()
    : Node("basic_subscriber") {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&BasicSubscriber::topic_callback, this, _1));
    }

 private:
    /**
     * @brief Callback function that is triggered upon receiving a new message on the subscribed topic.
     * @param msg The received message of type std_msgs::msg::String.
     */
    void topic_callback(const std_msgs::msg::String & msg) const;

   // Subscription object for the "topic" topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
