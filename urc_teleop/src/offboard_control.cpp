#include "offboard_control.hpp"

namespace offboard_control
{
  OffboardControl::OffboardControl(const rclcpp::NodeOptions &options)
      : rclcpp::Node("offboard_control", options)
  {
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(offboard_control::OffboardControl)