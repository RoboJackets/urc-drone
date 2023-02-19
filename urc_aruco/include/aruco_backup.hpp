#ifndef ARUCO_BACKUP_H
#define ARUCO_BACKUP_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <urc_msgs/msg/gps_location.hpp>
#include <urc_msgs/msg/gps_locations.hpp>

namespace aruco_backup
{
class ArucoBackup : public rclcpp::Node
{
public:
  explicit ArucoBackup(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<urc_msgs::msg::GPSLocations>::SharedPtr _waypoint_pub;
  rclcpp::Subscription<urc_msgs::msg::GPSLocation>::SharedPtr _pose_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _center_reached_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _aruco_detected_sub;

  int numPoints;
  double uncertaintyRadius, cameraFOV, detectionRadius;

  void poseCallback(const urc_msgs::msg::GPSLocation & msg);
  void centerReachedCallback(const std_msgs::msg::Bool & msg);
  void arucoDetectedCallback(const std_msgs::msg::Bool & msg);
};
}

#endif
