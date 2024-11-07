cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Specify the include directories
include_directories(include/beginner_tutorials)

# Add the talker executable
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Add the listener executable
add_executable(listener src/subscriber_member_function.cpp) 
ament_target_dependencies(listener rclcpp std_msgs)

# Install the targets
install(TARGETS      
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
  
# Build testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
