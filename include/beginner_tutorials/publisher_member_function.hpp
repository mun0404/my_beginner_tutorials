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

#include <beginner_tutorials/srv/new_str.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class BasicPublisher
 * @brief A basic publisher node class that publishes messages at a given frequency 
 * and provides a service to change the message content.
 */
class BasicPublisher : public rclcpp::Node {
 public:
    /**
     * @brief Constructs a new BasicPublisher object and initializes the 
     * publisher, timer, and service.
     * @param freq The frequency at which messages are published.
     */
    explicit BasicPublisher(double freq) : Node("basic_publisher"), count_(0) {
      double publisher_frequency =
                this->declare_parameter("publisher_frequency", freq);

      auto timer_interval =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / publisher_frequency));

      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

      pub_timer_ = this->create_wall_timer(
          timer_interval, std::bind(&BasicPublisher::timer_callback, this));

      service_ = this->create_service<beginner_tutorials::srv::NewStr>(
        "change_string", [this](
        const std::shared_ptr<beginner_tutorials::srv::NewStr::Request> request,
        std::shared_ptr<beginner_tutorials::srv::NewStr::Response> response) {
        change_string_service(request, response);
        });
    }

 private:
    /**
     * @brief Callback function triggered by the timer to publish messages at regular intervals.
     */
    void timer_callback();

    /**
     * @brief Service callback function to change the string message being published.
     * @param request The service request, which includes the new message string.
     * @param response The service response, indicating whether the message was successfully changed.
     */
    void change_string_service(
        const std::shared_ptr<beginner_tutorials::srv::NewStr::Request> request,
        std::shared_ptr<beginner_tutorials::srv::NewStr::Response> response);

    /**
     * @brief Retrieves the current message data string, including a counter.
     * @return A string combining the original message and a counter value.
     */
    std::string getData();

    // Timer for scheduling periodic message publishing
    rclcpp::TimerBase::SharedPtr pub_timer_;
    // Publisher for sending string messages on the specified topic
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // Counter used to differentiate published messages
    size_t count_;
    // Base string that is modified and published
    std::string original_string_ = "M Munawwar ROS 2 Programming A_2: ";
    // Service to change the message string
    rclcpp::Service<beginner_tutorials::srv::NewStr>::SharedPtr service_;
};
