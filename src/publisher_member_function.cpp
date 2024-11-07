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

void MinimalPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = "M Munawwar ROS 2 Programming A_1: "+std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
