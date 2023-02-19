#include "aruco_backup.hpp"

namespace aruco_backup
{

ArucoBackup::ArucoBackup(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_backup", options)
{
  _waypoint_pub = create_publisher<urc_msgs::msg::GPSLocation>(
    "~/backup_waypoints",
    rclcpp::SystemDefaultsQoS());

  _center_pose_sub = create_subscription<urc_msgs::msg::GPSLocation>(
    "~/center_pose", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::GPSLocation msg) {
      centerPoseCallback(msg);
    });

  _aruco_detected_sub = create_subscription<urc_msgs::msg::ArucoDetected>(
    "~/aruco_detected", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::ArucoDetected msg) {
      arucoDetectedCallback(msg);
    });

  numPoints = declare_parameter<int>("numPoints");
  uncertaintyRadius = declare_parameter<double>("uncertaintyRadius");
  cameraFOV = declare_parameter<double>("cameraFOV") * M_PI / 180;
  detectionRadius = declare_parameter<double>("detectionRadius");
  chordLength = 2 * detectionRadius * std::sin(cameraFOV / 2);
  spiralConstant = chordLength / (2 * M_PI);
}

void ArucoBackup::centerPoseCallback(const urc_msgs::msg::GPSLocation & msg)
{
  auto goalPose = urc_msgs::msg::GPSLocation();

  for (int i = 0; i < numPoints; ++i) {
    double theta = (i / (numPoints - 1)) * (uncertaintyRadius + (chordLength / 2)) / spiralConstant;
    double lat = (spiralConstant * theta * std::cos(theta) / metersToDegrees) + msg.lat;
    double lon = (spiralConstant * theta * std::sin(theta) / metersToDegrees) + msg.lon;

    goalPose.header.stamp = this->get_clock()->now();
    goalPose.lat = lat;
    goalPose.lon = lon;
    _waypoint_pub->publish(goalPose);
  }
}

void ArucoBackup::arucoDetectedCallback(const urc_msgs::msg::ArucoDetected & msg)
{

}
}

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_backup::ArucoBackup)
