#include "aruco_backup.hpp"

namespace aruco_backup
{

ArucoBackup::ArucoBackup(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_backup", options)
{
  _waypoint_pub = create_publisher<urc_msgs::msg::GPSLocation>(
    "~/backup_waypoints",
    rclcpp::SystemDefaultsQoS());

  _pose_sub = create_subscription<sensor_msgs::msg::Joy>(
    "~/joy", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::Joy msg) {
      joyCallback(msg);
    });

  updater_ptr = std::make_unique<diagnostic_updater::Updater>(this);
  updater_ptr->setHardwareID("ck_driverck");
  updater_ptr->add("Joystick Diagnostic", this, &ArucoBackup::joystick_diagnostic);

  leftJoyAxis = declare_parameter<int>("leftJoyAxis");
  rightJoyAxis = declare_parameter<int>("rightJoyAxis");
  leftInverted = declare_parameter<bool>("leftInverted");
  rightInverted = declare_parameter<bool>("rightInverted");
}

void ArucoBackup::joyCallback(const sensor_msgs::msg::Joy & msg)
{
  if (msg.buttons[1]) {
    maxVel -= maxVelIncr;
  } else if (msg.buttons[3]) {
    maxVel += maxVelIncr;
  }

  maxVel = std::max(std::min(maxVel, absoluteMaxVel), 0.0);
  set_parameter(rclcpp::Parameter("maxVel", maxVel));

  updater_ptr->force_update();

  auto cmd = urc_msgs::msg::VelocityPair();
  cmd.left_velocity = msg.axes[leftJoyAxis] * maxVel * (leftInverted ? -1.0 : 1.0);
  cmd.right_velocity = msg.axes[rightJoyAxis] * maxVel * (rightInverted ? -1.0 : 1.0);
  cmd.header.stamp = this->get_clock()->now();

  _waypoint_pub->publish(cmd);
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_backup::ArucoBackup)
