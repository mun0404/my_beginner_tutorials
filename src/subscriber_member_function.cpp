/**
 * @file subscriber_member_function.cpp
 * @author Mohammed Munawwar (mmunawwa@umd.edu)
 * @brief C++ program for simple ROS 2 subscriber member function
 * @version 0.1
 * @date 2024-11-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "../include/beginner_tutorials/subscriber_member_function.hpp"

/**
 * @brief Callback function to process received messages on the topic.
 *
 */
void BasicSubscriber::topic_callback(const std_msgs::msg::String & msg) const {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

/**
 * @brief Main function that initializes the ROS 2 subscriber node.
 *
 * @param argc Argument count
 * @param argv Argument vector
 * @return int Program exit status
 */
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
