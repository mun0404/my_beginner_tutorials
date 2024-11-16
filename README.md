[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# ROS 2 Beginner Tutorials - ENPM700
### Author : **Mohammed Munawwar**
This repository is a simple publisher-subscriber package developed for ROS 2 beginner tutorials.

## Building the Project

   ```bash
   # Source the ROS 2 setup script to configure your environment
   source /opt/ros/humble/setup.bash

   # Create a new ROS 2 workspace directory
   mkdir -p ~/my_beginner_tutorials/src && cd ~/my_beginner_tutorials/src

   # Clone the repository
   git clone https://github.com/mun0404/my_beginner_tutorials.git

   # Install package dependencies using rosdep
   cd ..
   rosdep install -i --from-path src --rosdistro humble -y

   # Build the package using colcon
   colcon build

   # Source the package
   source ./install/setup.bash
   ```

## Running the nodes

```bash
# To run the publisher node
ros2 run beginner_tutorials talker

# To run the subscriber node
ros2 run beginner_tutorials listener
```

## Style check

Perform these to ensure the quality of the code is maintained:
cppcheck, cpplint and clangd.

```bash
# Run cppcheck for code analysis
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $(find . -name "*.cpp" | grep -vE -e "^./build/") --check-config > results/cppcheck.txt

# Run cpplint for style checking
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp > results/cpplint.txt

# Run clang tidy
clang-tidy -extra-arg=-std=c++17 src/*.cpp

# To save the results of clang tidy
echo $? > results/clangtidy_output.txt

# To directly format the code according to the google style C++
clang-tidy -extra-arg=-std=c++17 src/*.cpp
```
## Services and Launch files
   
```bash
# Service call to modify published message.
ros2 service call /change_string beginner_tutorials/srv/NewStr "{new_message: changed message}"

# Using launch file.
ros2 launch beginner_tutorials publisher_subscriber_services.launch.py

# Using launch file to change the publisher frequncy from the default frequency of 2.0.
ros2 launch beginner_tutorials publisher_subscriber_services.launch.py freq:=0.5

# To view rqt_console.
ros2 run rqt_console rqt_console
```

## TF Frames

```bash
# Run the publisher in terminal
ros2 run beginner_tutorials talker

# Viewing the TF tree
ros2 run tf2_ros tf2_echo world talk

# In a new terminal 
ros2 topic echo /tf_static

# In another new terminal
ros2 run tf2_tools view_frames
```
## ROS2 Bags
```bash
# Run the launch file
ros2 launch beginner_tutorials publisher_subscriber_services.launch.py record_bag:=True

# Inspect the ros2 bag
ros2 bag info talkerbag

# Play back the ros2 bag
ros2 bag play talkerbag
```

## Catch2 test
```bash
# cd into ros2_integration test folder
cd ros2_ws/src/ros2_integration_test/

# In your ros workspace
colcon build --packages-select integration_test

# Run the launch file to run the test
ros2 launch integration_test integration_test.launch.yaml

# Run the test
colcon test --packages-select integration_test

# Save the results of the test
cat log/latest_test/integration_test/stdout_stderr.log > src/beginner_tutorials/results/catch2_tests_output.txt 
```