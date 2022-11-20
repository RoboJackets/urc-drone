//much of this code is imported from wheel odometer
#include "gate_behavior.hpp"

namespace gate_behavior
{

GateBehavior::GateBehavior(const rclcpp::NodeOptions & options)
: rclcpp::Node("gate_behavior", options)
{
  _enc_sub = create_subscription<urc_msgs::msg::VelocityPair>(
    "~/aruco",
    rclcpp::SystemDefaultsQoS(),
    [this](const urc_msgs::msg::VelocityPair msg)
    {GateBehavior::enc_callback(msg);});

  _odometry_pub = create_publisher<nav_msgs::msg::Odometry>(
    "~/gate",
    1000);


  // initialize position - map published is relative to position at time t=0
  x = 0;
  y = 0;
  yaw = 0;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(wheel_odometer::WheelOdometer)
