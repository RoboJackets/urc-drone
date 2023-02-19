#include "aruco_backup.hpp"

namespace aruco_backup
{

ArucoBackup::ArucoBackup(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_backup", options)
{
  _waypoint_pub = create_publisher<urc_msgs::msg::GPSLocations>(
    "~/backup_waypoints",
    rclcpp::SystemDefaultsQoS());

  _pose_sub = create_subscription<urc_msgs::msg::GPSLocation>(
    "~/gps_pose", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::GPSLocation msg) {
      poseCallback(msg);
    });

  _center_reached_sub = create_subscription<std_msgs::msg::Bool>(
    "~/gps_pose", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Bool msg) {
      centerReachedCallback(msg);
    });

  _aruco_detected_sub = create_subscription<std_msgs::msg::Bool>(
    "~/gps_pose", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Bool msg) {
      arucoDetectedCallback(msg);
    });

  numPoints = declare_parameter<int>("numPoints");
  uncertaintyRadius = declare_parameter<double>("uncertaintyRadius");
  cameraFOV = declare_parameter<double>("cameraFOV");
  detectionRadius = declare_parameter<double>("detectionRadius");
}

void ArucoBackup::poseCallback(const urc_msgs::msg::GPSLocation & msg)
{
  auto cmd = urc_msgs::msg::VelocityPair();
  cmd.left_velocity = msg.axes[leftJoyAxis] * maxVel * (leftInverted ? -1.0 : 1.0);
  cmd.right_velocity = msg.axes[rightJoyAxis] * maxVel * (rightInverted ? -1.0 : 1.0);
  cmd.header.stamp = this->get_clock()->now();

  _waypoint_pub->publish(cmd);
}

void ArucoBackup::centerReachedCallback(const std_msgs::msg::Bool & msg)
{

}

void ArucoBackup::arucoDetectedCallback(const std_msgs::msg::Bool & msg)
{

}
}

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_backup::ArucoBackup)
