#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/PointStamped.hpp"
#include <opencv2/core.hpp>

#include "../include/EyeTracking.hpp"

class tobii_gaze_node : public rclcpp::Node {
public:
    tobii_gaze_node(const char * node_name);

private:
    rclcpp::QoS sensor_qos_ = rclcpp::SensorDataQoS();
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gaze_publisher_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    EyeTracking eye_tracker;

    void timer_callback();
    geometry_msgs::msg::PointStamped convert_cp_point_to_ros(cv::point2f cv_point);
};
