#ifndef ARUCO_BACKUP_H
#define ARUCO_BACKUP_H

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urc_msgs/msg/aruco_detected.hpp>
#include <urc_msgs/msg/gps_location.hpp>

namespace aruco_backup
{
class ArucoBackup : public rclcpp::Node
{
public:
  explicit ArucoBackup(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<urc_msgs::msg::GPSLocation>::SharedPtr _waypoint_pub;
  rclcpp::Subscription<urc_msgs::msg::GPSLocation>::SharedPtr _center_pose_sub;
  rclcpp::Subscription<urc_msgs::msg::ArucoDetected>::SharedPtr _aruco_detected_sub;

  const int metersToDegrees = 111111;
  int numPoints;
  double uncertaintyRadius, cameraFOV, detectionRadius, chordLength, spiralConstant;

  void centerPoseCallback(const urc_msgs::msg::GPSLocation & msg);
  void arucoDetectedCallback(const urc_msgs::msg::ArucoDetected & msg);
};
}

#endif
