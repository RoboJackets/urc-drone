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

  //Aruco Tag Angles and Distance relative to camera.
  //TODO type correct?
  aruco_subscriber = create_subscription<urc_msgs::msg::ArucoDetection>(
    "/aruco", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::ArucoDetection
    arucomsg) {
      arucoCallback(arucomsg);
    });
  //GPS Location of Drone
  //TODO QoS Correct?
  gps_suscriber = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/data", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::NavSatFix gpsmsg) {
      gpsCallback(gpsmsg);
    });
  //Yaw, Pitch and Roll
  //TODO QoS Correct?
  orientation_subscriber = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::Imu imumsg) {
      orientationCallback(imumsg);
    });

  //orientationRead = false;
  //arucoRead = false;
  //gpsRead = false;
}


//TODO rework methods using sensor messages documentation
//in km
double ArucoLocation::getNextLatitude(double d, double xAngle, double yaw, double r) {
  if(!arucoRead || !gpsRead || !orientationRead) return -1.0;
  return asin(sin(droneLat)*cos(d / r) + cos(droneLat)*sin(d/r)*cos(xAngle + yaw));
}

//TODO diagnose
double ArucoLocation::getNextLongitude(double d, double xAngle, double yaw, double r) { //wouldn't this be xAngle+yaw
  if(!arucoRead || !gpsRead || !orientationRead) return -1.0;
  return droneLon + atan2(sin(xAngle + yaw)*sin(d/r)*cos(droneLat), cos(d/R)-sin(droneLat)*sin(getNextLatitude(d, xAngle, yaw, r)));
}

double ArucoLocation::findD(double trueD, double yAngle, double pitch){
  if(!arucoRead || !gpsRead || !orientationRead) return -1.0;
  return trueD * cos(pitch + yAngle);
}







//TODO rework callbacks
//Guess: Publishes a message, use pointers to access required values inside callback
//needs to read all 3 topics before implementation
void ArucoLocation::arucoCallback(const urc_msgs::msg::ArucoDetection & arucomsg) {
  // aruco
  /*
  this->xAngle = xAngle;
  this->yAngle = yAngle;
  this->trueDist = trueDist;
  arucoRead = true;
  */
}

void ArucoLocation::gpsCallback(const sensor_msgs::msg::NavSatFix & gpsmsg) {
  /*
  this->curX = curX;
  this->curY = curY;
  this->curZ = curZ;
  gpsRead = true;
  */
}

void ArucoLocation::orientationCallback(const sensor_msgs::msg::Imu & imumsg) {
  /*
  this->yaw = yaw;
  this->pitch = pitch;
  this->roll = roll;
  orientationRead = true;
  */
}

}
RCLCPP_COMPONENTS_REGISTER_NODE(aruco_detector::ArucoDetector)
