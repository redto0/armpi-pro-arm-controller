cmake_minimum_required(VERSION 3.8)
project(bus_servo_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiServoCommand.msg"
  "msg/ServoCommand.msg"
  "msg/ServoStatus.msg"
  DEPENDENCIES
  std_msgs
)

# Install Python modules
install(PROGRAMS
  src/bus_servo_controller/bus_servo_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
