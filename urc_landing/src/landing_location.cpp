// The main idea of this node is that, given an array of possible landing locations (array) and the location of an aruco marker, choose the landing location closest to the aruco marker.
#include "landing_location.hpp"

namespace landing_location
{

LandingLocation::LandingLocation(const rclcpp::NodeOptions & options)
: rclcpp::Node("landing_location", options)
{
  landing_publisher = create_publisher<urc_msgs::msg::LandingLocation>(
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
  possibleLandingLocations = location_array_msg.landing_locations;
}
void LandingLocation::arucoCallback(
  const urc_msgs::msg::ArucoLocation & aruco_msg)
{
  if (possibleLandingLocations == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Landing locations should have been instantiated at this point!");
    return;
  }
  std::vector<urc_msgs::msg::LandingLocation> vectorizedLandingLocations();
  for (urc_msgs::msg::LandingLocation landingLocation: possibleLandingLocations) {
    vectorizedLandingLocations.push_back(landingLocation);
  }
  int closestLocationIndex = 0;
  // const int numElements = (sizeof(possibleLandingLocations) / sizeof(possibleLandingLocations[0]));
  for (int i = 1; i < vectorizedLandingLocations.size() ; ++i) {
    if (sqrt(pow(abs(vectorizedLandingLocations[i].lon - aruco_msg.lon), 2) + pow(abs(vectorizedLandingLocations[i].lat - aruco_msg.lat), 2)) < 
      sqrt(pow(abs(vectorizedLandingLocations[closestLocationIndex].lon - aruco_msg.lon), 2) + pow(abs(vectorizedLandingLocations[closestLocationIndex].lat - aruco_msg.lat), 2)))
      {
        closestLocationIndex = i;
      }
  }
  landing_publisher->publish(possibleLandingLocations[closestLocationIndex]);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(landing_location::LandingLocation);