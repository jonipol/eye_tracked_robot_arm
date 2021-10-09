cmake_minimum_required(VERSION 3.5)
project(tobii_driver)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

#include(FetchContent)
#FetchContent_Declare(json
#        GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent
#        GIT_TAG v3.10.2)
#FetchContent_GetProperties(json)
#if(NOT json_POPULATED)
#  FetchContent_Populate(json)
#  add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
#endif()

install(DIRECTORY
        include/
        DESTINATION share/${PROJECT_NAME}/
        )

ament_export_include_directories(
        include
)
# Include Cpp "include" directory
include_directories(include)

add_library(camera_calibrator SHARED
        src/CameraCalibrator.cpp)
ament_target_dependencies(camera_calibrator OpenCV)

install(
        TARGETS camera_calibrator
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# Create Cpp executable
#target_link_libraries(xxx nlohmann_json::nlohmann_json)

add_executable(tobii_camera src/tobii_camera.cpp)
ament_target_dependencies(tobii_camera rclcpp OpenCV sensor_msgs cv_bridge)

#add_executable(tobii_gaze src/tobii_gaze.cpp)
#ament_target_dependencies(tobii_gaze rclcpp geometry_msgs)



# Install Cpp executables
install(TARGETS
#        tobii_node
        tobii_camera
        DESTINATION lib/${PROJECT_NAME}
        )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        DESTINATION lib/${PROJECT_NAME}
        )
ament_package()
