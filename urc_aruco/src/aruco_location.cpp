#include "aruco_location.hpp"

namespace aruco_location
{
    
ArucoLocation::ArucoLocation(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_location", options)
{

  location_publisher = create_publisher<double*>(
    "~/aruco/tag_location",
    rclcpp::SystemDefaultsQoS()
  );

  //angles and true dist
  aruco_subscriber = create_subscription<urc_msgs::msg::ArucoLocation>( //todo fix, this shouldn't be arucoLoc
    "~/aruco/", rclcpp::SystemDefaultsQoS(), [this](double xAngle, double yAngle, double trueDist) {
      arucoCallback(xAngle, yAngle, trueDist);
    });
  //cur drone loc
  gps_suscriber = create_subscription<urc_msgs::msg::ArucoLocation>(
    "~/gps/", rclcpp::SystemDefaultsQoS(), [this](double curX, double curY, double curZ) {
      gpsCallback(curX, curY, curZ);
    });
  //yaw, pitch, roll
  orientation_subscriber = create_subscription<urc_msgs::msg::ArucoLocation>(
    "~/orientation/", rclcpp::SystemDefaultsQoS(), [this](double yaw, double pitch, double roll) {
      orientationCallback(yaw, pitch, roll);
    });

  orientationRead = false;
  arucoRead = false;
  gpsRead = false;
}

//in km
double ArucoLocation::getNextLatitude(double d, double xAngle, double yaw, double r) {
  if(!arucoRead || !gpsRead || !orientationRead) return -1.0;
  return asin(sin(droneLat)*cos(d / r) + cos(droneLat)*sin(d/r)*cos(xAngle + yaw));
}

double ArucoLocation::getNextLongitude(double d, double xAngle, double yaw, double r) { //wouldn't this be xAngle+yaw
  if(!arucoRead || !gpsRead || !orientationRead) return -1.0;
  return droneLon + atan2(sin(xAngle + yaw)*sin(d/r)*cos(droneLat), cos(d/R)-sin(droneLat)*sin(getNextLatitude(d, xAngle, yaw, r))); //er fix this extra call
}

double ArucoLocation::findD(double trueD, double yAngle, double pitch){
  if(!arucoRead || !gpsRead || !orientationRead) return -1.0;
  return trueD * cos(pitch + yAngle);
}

//needs to read all 3 topics before implementation
void ArucoLocation::arucoCallback(double xAngle, double yAngle, double trueDist) {
  // arucoo
  this->xAngle = xAngle;
  this->yAngle = yAngle;
  this->trueDist = trueDist;
  arucoRead = true;
}

void ArucoLocation::gpsCallback(double curX, double curY, double curZ) {
  this->curX = curX;
  this->curY = curY;
  this->curZ = curZ;
  gpsRead = true;
}

void ArucoLocation::orientationCallback(double yaw, double pitch, double roll) {
  this->yaw = yaw;
  this->pitch = pitch;
  this->roll = roll;
  orientationRead = true;
}

}