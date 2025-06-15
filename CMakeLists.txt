cmake_minimum_required(VERSION 3.8)
project(bus_servo_controller)

# Find ALL dependencies FIRST
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiServoCommand.msg"
  "msg/ServoCommand.msg"
  "msg/ServoStatus.msg"
  DEPENDENCIES std_msgs
)

# Install Python files (modified)
install(DIRECTORY
  src/bus_servo_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/bus_servo_controller/bus_servo_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()