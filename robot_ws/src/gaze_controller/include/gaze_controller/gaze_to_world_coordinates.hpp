//
// Created by jp on 20.10.2021.
//
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>


#ifndef ROBOT_WS_GAZE_TO_WORLD_COORDINATES_HPP
#define ROBOT_WS_GAZE_TO_WORLD_COORDINATES_HPP


class gaze_to_world_coordinates : public rclcpp::Node {
public:
    gaze_to_world_coordinates(const char * node_name);
    virtual ~gaze_to_world_coordinates() {};
    geometry_msgs::msg::Point calculate_world_point_from_gaze(apriltag_msgs::msg::AprilTagDetectionArray &tag_array);
    void extract_camera_intrinsics(sensor_msgs::msg::CameraInfo camera_info, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
    geometry_msgs::msg::Point point3f_to_ros_point(cv::Point3f cv_point);
    cv::Point2f ros_point_to_pixel(geometry_msgs::msg::PointStamped ros_point);

private:
    rclcpp::QoS sensor_qos_ = rclcpp::SensorDataQoS();
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gaze_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr world_point_pub_;

    void gaze_callback(geometry_msgs::msg::PointStamped::SharedPtr gaze_point);
    void camera_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
    void tag_callback(apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr tag_array);

    geometry_msgs::msg::PointStamped latest_gaze_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    std::vector<cv::Point3f> modelPts_;

    // TODO: Make smaller! Big just for testing and initial implementation
    float time_tolerance_ = 1.0;     // Time allowed to be passed since latest gaze measurement
};


#endif //ROBOT_WS_GAZE_TO_WORLD_COORDINATES_HPP
