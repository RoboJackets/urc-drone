#include "gate_behavior.hpp"

namespace gate_behavior
{

GateBehavior::GateBehavior(const rclcpp::NodeOptions & options)
: rclcpp::Node("gate_behavior", options)
{
    aruco_location = create_subscription<urc_msgs::msg::aruco>(
    "~/aruco",
    rclcpp::SystemDefaultsQoS(),
    [this](float lon, double lat)
    {aurcoCallback(lon, lat);});

    possible_locations = create_subscription<urc_msgs::msg::possibleLocations>(
    "~/possibleLocations",
    rclcpp::SystemDefaultsQoS(),
    [this](float[][] possibleLocations)
    {locationCallback(possibleLocations);});

  _odometry_pub = create_publisher<urc_msgs::msg::landLocation>(
    "~/landlocation");

  arucoRead = false;
  locationRead = false;
}

void GateBehavior::arucoCallback(float lon, float lat) {
  this->lon = lon;
  this->lat = lat;
  arucoRead = true;
}

void GateBehavior::locationCallback(float[][] possibleLocations) {
  this->lon = possibleLocations[0];
  this->lat = possibleLocations[1];
  locationRead = true;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(gate_behavior::GateBehavior)
