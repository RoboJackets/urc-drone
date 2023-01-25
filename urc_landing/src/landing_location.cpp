// The main idea of this node is that, given an array of possible landing locations (array) and the location of an aruco marker, choose the landing location closest to the aruco marker.
#include "landing_location.hpp"

namespace landing_location
{

LandingLocation::LandingLocation(const rclcpp::NodeOptions & options)
: rclcpp::Node("landing_location", options)
{
  //instantiate attributes here
  this.possibleLandingLocations = null;

  landing_publisher = create_publisher<urc_msgs::msg::ArucoDetection>(
    "~/aruco",
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
  const std::vector<urc_msgs::msg::LandingLocation> & landingLocationArr)
{
  this.possibleLandingLocations = landingLocationArr;
}
void LandingLocation::arucoCallback(
  const urc_msgs::msg::ArucoLocation & arucoLocation)
{
  if (this.possibleLandingLocations != null) {
    int closestLocationIndex = 0;
    for(int i = 0; i < this.possibleLandingLocations.size() ; i++) {
      if (
        sqrt(this.possibleLandingLocations[i].lon, 2) + sqrt(this.possibleLandingLocations[i].lat, 2) < 
        sqrt(this.possibleLandingLocations[closestLocationIndex].lon, 2) + sqrt(this.possibleLandingLocations[closestLocationIndex].lat, 2)
        ) {
          closestLocationIndex = i;
        }
    }
    landing_publisher->publish(this.possibleLandingLocations[closestLocationIndex]);
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(landing_location::LandingLocation);
