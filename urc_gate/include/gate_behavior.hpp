//much of this code is imported from wheel odometer
#ifndef GATE_BEHAVIOR_H
#define GATE_BEHAVIOR_H

#include <rclcpp/rclcpp.hpp>

namespace gate_behavior
{
class GateBehavior : public rclcpp::Node
{
public:
  explicit GateBehavior(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<urc_msgs::msg::VelocityPair>::SharedPtr _enc_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odometry_pub;

  double wheel_sep;

  float x;
  float y;
  float yaw;

  void enc_callback(const urc_msgs::msg::VelocityPair & msg);
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
};
}

#endif