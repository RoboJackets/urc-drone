#include "aruco_location.hpp"

namespace aruco_location
{
    
ArucoLocation::ArucoLocation(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_location", options)
{

  location_publisher = create_publisher<urc_msgs::msg::ArucoLocation>(
    "~/aruco",
    rclcpp::SystemDefaultsQoS()
  );

  
  aruco_subscriber = create_subscription<urc_msgs::msg::ArucoLocation>(
    "~/aruco", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::ArucoLocation msg) {
      arucoCallback(msg);
    });
  gps_suscriber = create_subscription<urc_msgs::msg::ArucoLocation>(
    "~/aruco", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::ArucoLocation msg) {
      gpsSubscriber(msg);
    });
  orientation_subscriber = create_subscription<urc_msgs::msg::ArucoLocation>(
    "~/aruco", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::ArucoLocation msg) {
      orientationSubscriber(msg);
    });
}

double ArucoLocation::getNextLatitude(double d, double xAngle, double yaw, double r) {
  return asin(sin(droneLat)*cos(d / r) + cos(droneLat)*sin(d/r)*cos(xAngle + yaw));
}

double ArucoLocation::getNextLongitude(double d, double xAngle, double yaw, double r) { //wouldn't this be xAngle+yaw
  return droneLon + atan2(sin(xAngle + yaw)*sin(d/r)*cos(droneLat), cos(d/R)-sin(droneLat)*sin(getNextLatitude(d, xAngle, yaw, r))); //er fix this extra call
}

double findD(double trueD, double yAngle, double pitch){
  return trueD * cos(pitch + yAngle);
}

}