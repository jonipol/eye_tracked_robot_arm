//
// Created by jp on 2.10.2021.
//
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "../include/tobii_camera.hpp"
#include "CameraCalibrator.hpp"

#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/header.hpp"

using std::literals::chrono_literals::operator""ms;

tobii_camera::tobii_camera(const char * node_name) : Node(node_name) {
    this->declare_parameter("camera_id", 0);
    this->declare_parameter("flip_code", 1);    // OpenCV flipCode

    camera_id_ = this->get_parameter("camera_id").as_int();
    flip_code_ = this->get_parameter("flip_code").as_int();

    inputCapture_.open(camera_id_);
    // Check calibration -> calibrate if needed
    //CameraCalibrator cameraCalibrator = CameraCalibrator(10);
    //cameraCalibrator.calibrate(inputCapture_);


    camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera", sensor_qos_);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 1);
    pub_timer_ = this->create_wall_timer(1ms, std::bind(&tobii_camera::timer_callback, this));
}

void tobii_camera::timer_callback() {
    if (inputCapture_.grab()) {
        inputCapture_.retrieve(latest_image_);
        cv::resize(latest_image_, latest_image_, cv::Size(640, 480));
        cv::flip(latest_image_, latest_image_, flip_code_);
        cv::waitKey(1);     // Raises rate from 1 to 7 on 640x480 image!?!
        camera_publisher_->publish(convertMatToImageMsg(latest_image_));
        sensor_msgs::msg::CameraInfo cam_info;
        cam_info.width = 640;
        cam_info.height = 480;
        cam_info.header.frame_id = "tobii_cam";
        camera_info_publisher_->publish(cam_info);
    }
    else {
        RCLCPP_WARN(get_logger(), "Video input is not ready.");
    }
}

sensor_msgs::msg::Image tobii_camera::convertMatToImageMsg(cv::Mat &image) {
    sensor_msgs::msg::Image image_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::msg::Header header;

    header.frame_id = "tobii_cam";
    header.stamp = this->get_clock()->now();

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    img_bridge.toImageMsg(image_msg);
    return image_msg;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto cam_node = std::make_shared<tobii_camera>("tobii_camera");
    rclcpp::spin(cam_node);
    rclcpp::shutdown();
}