cmake_minimum_required(VERSION 3.5)
project(gaze_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
#find_package(moveit2 REQUIRED)

find_package(OpenCV REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Include Cpp "include" directory
include_directories(include)

set(gaze_deps
  rclcpp
  geometry_msgs
  sensor_msgs
  apriltag_msgs
  OpenCV
)

set(arm_deps
  rclcpp
  geometry_msgs

)

add_executable(gaze_to_world_coordinates src/gaze_to_world_coordinates.cpp)
ament_target_dependencies(gaze_to_world_coordinates ${gaze_deps})

add_executable(arm_controller src/arm_controller.cpp)
ament_target_dependencies(arm_controller ${arm_deps})

install(TARGETS
  gaze_to_world_coordinates
  arm_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  gaze_controller/gaze_controller.py
  gaze_controller/bounding_box.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
