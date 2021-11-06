#include "gaze_controller/arm_controller.hpp"

using std::placeholders::_1;

arm_controller::arm_controller(const char * node_name) : Node(node_name) {
    this->declare_parameter("coordinate_topic", "/world_point");
    std::string coordinate_topic = this->get_parameter("coordinate_topic").as_string();

    coordinate_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            coordinate_topic, sensor_qos_, std::bind(&coordinate_callback, this, _1));

    move_group(this, PLANNING_GROUP_);

    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);

      // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_.getJointModelGroupNames().begin(), move_group_.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
}

void arm_controller::coordinate_callback(geometry_msgs::msg::Point::SharedPtr coordinate) {
    // Pass coordinate to moveit to make robot move
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto arm_controller_node = std::make_shared<arm_controller>("arm_controller");
    rclcpp::spin(arm_controller);
    rclcpp::shutdown();
}
