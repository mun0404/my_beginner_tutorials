/**
 * @file publisher_member_function.cpp
 * @author Mohammed Munawwar (mmunawwa@umd.edu)
 * @brief C++ program for simple ROS 2 publisher member function
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2024
 * 
 * @license MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
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

  static tf2_ros::TransformBroadcaster tf_broadcaster(this);
  geometry_msgs::msg::TransformStamped tfs;

  tfs.header.stamp = this->now();
  tfs.header.frame_id = "world";  // Parent frame
  tfs.child_frame_id = "talk";    // Child frame

  // Set non-zero translation and rotation
  tfs.transform.translation.x = 1.0;
  tfs.transform.translation.y = 1.0;
  tfs.transform.translation.z = 1.0;
  tfs.transform.rotation.x = 0.1;
  tfs.transform.rotation.y = 0.1;
  tfs.transform.rotation.z = 0.1;
  tfs.transform.rotation.w = 0.1;

  // Broadcast the transform
  tf_broadcaster.sendTransform(tfs);
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
