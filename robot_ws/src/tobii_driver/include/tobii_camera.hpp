//
// Created by jp on 2.10.2021.
//
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#ifndef ROBOT_WS_TOBII_CAMERA_HPP
#define ROBOT_WS_TOBII_CAMERA_HPP


class tobii_camera : public rclcpp::Node {
public:
    tobii_camera(const char * node_name);

private:
    void timer_callback();
    cv::VideoCapture inputCapture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::QoS sensor_qos_ = rclcpp::SensorDataQoS();
    int camera_id_;
    int flip_code_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    cv::Mat latest_image_;
    sensor_msgs::msg::Image convertMatToImageMsg(cv::Mat &image);
};


#endif //ROBOT_WS_TOBII_CAMERA_HPP
