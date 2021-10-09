#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "../include/tobii_gaze_node.hpp"

using std::literals::chrono_literals::operator""ms;

tobii_gaze_node::tobii_gaze_node(const char * node_name) : Node(node_name) {
    gaze_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/gaze", sensor_qos_);
    pub_timer_ = this->create_wall_timer(1ms, std::bind(&tobii_gaze_node::timer_callback, this));
}

void tobii_gaze_node::timer_callback() {
    cv::point2f gaze_point = eye_tracker.getGazePoint();
    gaze_publisher_->publish(tobii_gaze_node::convert_cp_point_to_ros(gaze_point));
}

geometry_msgs::msg::PointStamped tobii_gaze_node::convert_cp_point_to_ros(cv::point2f cv_point) {
    geometry_msgs::msg::PointStamped point_msg;
    // TODO: Not safe way to handle the timestamp. Should get stamp when the gaze is measured in EyeTracking.cpp
    point_msg.header.stamp = this->get_clock()->now();
    point_msg.point.x = cv_point.x;
    point_msg.point.y = cv_point.y;
    return point_msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto gaze_node = std::make_shared<tobii_camera>("tobii_gaze_node");
    rclcpp::spin(gaze_node);
    rclcpp::shutdown();
}