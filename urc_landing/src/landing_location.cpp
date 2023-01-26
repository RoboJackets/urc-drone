// The main idea of this node is that, given an array of possible landing locations (array) and the location of an aruco marker, choose the landing location closest to the aruco marker.
#include "landing_location.hpp"

namespace landing_location
{

LandingLocation::LandingLocation(const rclcpp::NodeOptions & options)
: rclcpp::Node("landing_location", options)
{
  landing_publisher = create_publisher<urc_msgs::msg::PossibleLandingLocations>(
    "~/landing_location",
    rclcpp::SystemDefaultsQoS()
  );
  aruco_locations_subscriber = create_subscription<urc_msgs::msg::ArucoLocation>(
    "~/tag_location", rclcpp::SensorDataQoS(), [this](const urc_msgs::msg::ArucoLocation aruco_location) {
      arucoCallback(arucoLocation);
    });
  possible_landing_locations_subscribers = create_subscription<std::vector<urc_msgs::msg::LandingLocation>>(
  "~/PLACEHOLDERFORSOMELANDINGLOCATIONSSHIT", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::ArucoDetection
  location_array) {
    landing_locations_callback(location_array);
  });
}

void LandingLocation::landingLocationsArrCallback(
  const urc_msgs::msg::PossibleLandingLocations & landingLocationsMsg)
{
  this.possibleLandingLocations = landingLocationsMsg.landing_locations;
}
void LandingLocation::arucoCallback(
  const urc_msgs::msg::ArucoLocation & arucoLocation)
{
  if (possibleLandingLocations == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Landing locations should have been instantiated at this point!");
    return;
  }

  int closestLocationIndex = 0;
  for (int i = 1; i < (sizeof(possibleLandingLocations) / sizeof(possibleLandingLocations[0])); ++i) {
    if (sqrt(pow(this.possibleLandingLocations[i].lon, 2) + pow(this.possibleLandingLocations[i].lat, 2)) < 
      sqrt(pow(this.possibleLandingLocations[closestLocationIndex].lon, 2) + pow(this.possibleLandingLocations[closestLocationIndex].lat, 2)))
      {
        closestLocationIndex = i;
      }
  }
  landing_publisher->publish(this.possibleLandingLocations[closestLocationIndex]);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(landing_location::LandingLocation);
