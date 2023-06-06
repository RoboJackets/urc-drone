// The main idea of this node is that, given an array of possible landing locations (array) and the location of an aruco marker, choose the landing location closest to the aruco marker.
#include "landing_location.hpp"

namespace landing_location
{

LandingLocation::LandingLocation(const rclcpp::NodeOptions & options)
: rclcpp::Node("landing_location", options)
{
  landing_publisher = create_publisher<urc_msgs::msg::ChosenLandingLocation>(
  "~/landing_location",
    rclcpp::SystemDefaultsQoS()
  );
  aruco_locations_subscriber = create_subscription<urc_msgs::msg::ArucoLocation>(
  "~/tag_location", rclcpp::SensorDataQoS(), [this](const urc_msgs::msg::ArucoLocation aruco_msg) {
      arucoCallback(aruco_msg);
    });
  possible_landing_locations_subscriber = create_subscription<urc_msgs::msg::PossibleLandingLocations>(
  "~/possible_landing_locations", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::PossibleLandingLocations
  location_array_msg) {
    landingLocationsCallback(location_array_msg);
  });
}

void LandingLocation::landingLocationsCallback(
  const urc_msgs::msg::PossibleLandingLocations & location_array_msg)
{
  int size = sizeof(possibleLandingLats)/sizeof(possibleLandingLats[0]);
  for(int i = 0; i < size; i++) {
    possibleLandingLats[i] = location_array_msg.possible_landing_lats[i];
    possibleLandingLons[i] = location_array_msg.possible_landing_lons[i];
  }
}
void LandingLocation::arucoCallback(
  const urc_msgs::msg::ArucoLocation & aruco_msg)
{
  if (possibleLandingLats == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Landing locations should have been instantiated at this point!");
    return;
  }
  int closestLocationIndex = 0;
  arrayLength = (sizeof(possibleLandingLats) / sizeof(possibleLandingLats[0]));
  for (int i = 1; i < arrayLength; ++i) {
    if (sqrt(pow(abs(possibleLandingLats[i] - aruco_msg.lat), 2) + pow(abs(possibleLandingLats[i] - aruco_msg.lat), 2)) < 
      sqrt(pow(abs(possibleLandingLons[closestLocationIndex] - aruco_msg.lon), 2) + pow(abs(possibleLandingLons[closestLocationIndex] - aruco_msg.lon), 2)))
      {
        closestLocationIndex = i;
      }
  }


    urc_msgs::msg::ChosenLandingLocation location_message;
    location_message.header.stamp = aruco_msg.header.stamp;
    location_message.lon = possibleLandingLons[closestLocationIndex];
    location_message.lat = possibleLandingLats[closestLocationIndex];
    landing_publisher->publish(location_message);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(landing_location::LandingLocation);