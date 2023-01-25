#ifndef LANDING_LOCATION_H
#define LANDING_LOCATION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urc_msgs/msg/aruco_detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bits/stdc++.h>
#include <image_transport/image_transport.hpp>
#include <cmath> //for power function

namespace landing_location
{

class LandingLocation : public rclcpp::Node
{
public:
  explicit LandingLocation(const rclcpp::NodeOptions & options);

private:
  std::vector<urc_msgs::msg::LandingLocation> possibleLandingLocations;
  rclcpp::Publisher<urc_msgs::msg::LandingLocation>::SharedPtr landing_publisher; //message type placeholder
  rclcpp::Subscription<urc_msgs::msg::ArucoLocation>::SharedPtr aruco_locations_subscriber;
  rclcpp::Subscription<std::vector<urc_msgs::msg::LandingLocation>>::SharedPtr possible_locations_subscriber;
  void aruco_locations_callback(const ArucoLocation location_msg);
  void landing_locations_callback(const std::vector<LandingLocation> location_array);
};
}

#endif
