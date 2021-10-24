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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
# find_package(PkgConfig REQUIRED)

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

# these calls create special `PkgConfig::<MODULE>` variables
# pkg_check_modules(tobii_research REQUIRED IMPORTED_TARGET tobii_research)

# add_library(eye_tracking SHARED
#        src/EyeTracking.cpp)
# ament_target_dependencies(eye_tracking OpenCV)
# target_link_libraries(eye_tracking PkgConfig::tobii_research)

install(
        TARGETS camera_calibrator
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# Create Cpp executable
#target_link_libraries(xxx nlohmann_json::nlohmann_json)

set(dependencies
    rclcpp
    std_msgs
    sensor_msgs
    OpenCV
    cv_bridge
)

set(gaze_dependencies
    rclcpp
    geometry_msgs
)

add_executable(tobii_camera src/tobii_camera.cpp)
ament_target_dependencies(tobii_camera ${dependencies})

add_executable(tobii_gaze_node src/tobii_gaze_node.cpp)
ament_target_dependencies(tobii_gaze_node ${gaze_dependencies})

# Install Cpp executables
install(TARGETS
        tobii_camera
        tobii_gaze_node
        DESTINATION lib/${PROJECT_NAME}
        )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
        DESTINATION lib/${PROJECT_NAME}
        )
ament_package()
