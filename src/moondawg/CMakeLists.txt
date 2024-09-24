cmake_minimum_required(VERSION 3.5)
project(moondawg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_python)

# Install Python executable
install(PROGRAMS
  src/gps_publisher.py  # Update with the correct path to your Python file
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package macros
ament_package()
