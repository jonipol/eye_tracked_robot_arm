#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"


class arm_controller : public rclcpp::Node {
public:
    arm_controller(const char * node_name);
    virtual ~arm_controller() {};
private:
    rclcpp::QoS sensor_qos_ = rclcpp::SensorDataQoS();
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coordinate_sub_;

    void coordinate_callback(geometry_msgs::msg::Point::SharedPtr coordinate);
}