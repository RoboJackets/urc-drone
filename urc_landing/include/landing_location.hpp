#ifndef LANDING_LOCATION_H
#define LANDING_LOCATION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urc_msgs/msg/aruco_detection.hpp>
#include <urc_msgs/msg/aruco_location.hpp>
#include <urc_msgs/msg/possible_landing_locations.hpp>
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
  urc_msgs::msg::PossibleLandingLocations possibleLandingLocations;
  rclcpp::Publisher<urc_msgs::msg::LandingLocation>::SharedPtr landing_publisher; //message type placeholder
  rclcpp::Subscription<urc_msgs::msg::ArucoLocation>::SharedPtr aruco_locations_subscriber;
  rclcpp::Subscription<urc_msgs::msg::PossibleLandingLocations>::SharedPtr possible_landing_locations_subscriber;
  void arucoCallback(const urc_msgs::msg::ArucoLocation & aruco_msg);
  void landingLocationsCallback(const urc_msgs::msg::PossibleLandingLocations & location_array_msg);
};
}

#endif
