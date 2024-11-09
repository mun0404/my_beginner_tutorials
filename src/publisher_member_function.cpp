/**
 * @file publisher_member_function.cpp
 * @author Mohammed Munawwar (mmunawwa@umd.edu)
 * @brief C++ program for simple ROS 2 publisher member function
 * @version 0.1
 * @date 2024-11-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "../include/beginner_tutorials/publisher_member_function.hpp"

/**
 * @brief Publishes a message on the specified topic at each timer callback interval.
 */
void BasicPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = getData();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("basic_publisher"),
                     "Publishing: " << message.data.c_str());
  publisher_->publish(message);
}

/**
 * @brief Service callback function that changes the message string
 * based on the request and provides a response.
 * 
 * @param request The request containing the new message string.
 * @param response The response indicating if the string change was successful.
 */
void BasicPublisher::change_string_service(
      const std::shared_ptr<beginner_tutorials::srv::NewStr::Request> request,
      std::shared_ptr<beginner_tutorials::srv::NewStr::Response> response) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("basic_publisher"),
                     "Modifying String - " << request->new_message);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("basic_publisher"),
                      "Modifying String - " << request->new_message);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("basic_publisher"),
                     "Modifying String - " << request->new_message);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("basic_publisher"),
                      "Modifying String - " << request->new_message);
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("basic_publisher"),
                      "Modifying String - " << request->new_message);
  original_string_ = request->new_message;
  response->response = true;
}

/**
 * @brief Retrieves the current message data with an incremented counter.
 * 
 * @return The formatted message data as a std::string.
 */
std::string BasicPublisher::getData() {
  return original_string_ + " - " + std::to_string(count_++);
}

/**
 * @brief Main function that initializes the ROS 2 node and starts the publisher.
 * 
 * @param argc Argument count.
 * @param argv Argument values.
 * @return int Exit code.
 */
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  // Default frequency value
  double frequency = 2.0;
  // Check if a custom frequency is provided as a command-line argument
  if (argc > 1) {
    frequency = std::atof(argv[1]);
  }

  rclcpp::spin(std::make_shared<BasicPublisher>(frequency));
  rclcpp::shutdown();
  return 0;
}
