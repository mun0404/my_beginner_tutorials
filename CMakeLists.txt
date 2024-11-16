cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
    std_msgs
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  #UPDATED
find_package(tf2 REQUIRED)  #UPDATED
find_package(tf2_ros REQUIRED)  #UPDATED

include_directories(include/beginner_tutorials)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NewStr.srv"
  DEPENDENCIES std_msgs
)


# Add the talker executable
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)

# Add the listener executable
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Export generated typesupport
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

#target_link_libraries(listener "${cpp_typesupport_target}")
target_link_libraries(talker "${cpp_typesupport_target}")

# Install the executables
install(TARGETS      
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/test.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_package()
