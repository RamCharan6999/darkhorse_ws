cmake_minimum_required(VERSION 3.8)
project(darkhorse_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/camera_node.py
  ${PROJECT_NAME}/radar_rx_node.py
  ${PROJECT_NAME}/radar_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Resource folders (in case you prefer CMake to copy as well)
install(DIRECTORY
  launch
  config
  rviz
  models
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.yaml" PATTERN "*.yml"
    PATTERN "*.rviz"
    PATTERN "*.onnx" PATTERN "*.pt" PATTERN "*.txt"
)

ament_package()

