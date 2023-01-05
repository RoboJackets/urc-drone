#include "aruco_location.hpp"

namespace aruco_location
{
ArucoLocation::ArucoLocation(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_location", options)
{
  //Publisher
  location_publisher = create_publisher<urc_msgs::msg::ArucoLocation>(
    "~/tag_location",
    rclcpp::SystemDefaultsQoS()
  );

  //Aruco Tag Angles and Distance relative to camera.
  aruco_subscriber = create_subscription<urc_msgs::msg::ArucoDetection>(
    "/aruco", rclcpp::SystemDefaultsQoS(), [this](const urc_msgs::msg::ArucoDetection
    aruco_msg) {
      arucoCallback(aruco_msg);
    });

  //GPS Location of Drone
  gps_subscriber = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/data", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::NavSatFix gps_msg) {
      gpsCallback(gps_msg);
    });

  //Yaw, Pitch and Roll (Orientation)
  orientation_subscriber = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu imu_msg) {
      orientationCallback(imu_msg);
    });
}


//TODO rework methods using sensor messages documentation in km
double ArucoLocation::getNextLatitude(double d, double xAngle, double yaw, double r)
{
  if (!arucoRead || !gpsRead || !orientationRead) {
    return -1.0;
  }
  return asin(sin(droneLat) * cos(d / r) + cos(droneLat) * sin(d / r) * cos(xAngle + yaw));
}

//TODO diagnose
double ArucoLocation::getNextLongitude(double d, double xAngle, double yaw, double r)   //wouldn't this be xAngle+yaw
{
  if (!arucoRead || !gpsRead || !orientationRead) {
    return -1.0;
  }
  return droneLon + atan2(
    sin(xAngle + yaw) * sin(d / r) * cos(droneLat), cos(d / r) - sin(
      droneLat) * sin(getNextLatitude(d, xAngle, yaw, r)));
}

double ArucoLocation::findD(double trueD, double yAngle, double pitch)
{
  if (!arucoRead || !gpsRead || !orientationRead) {
    return -1.0;
  }
  return trueD * cos(pitch + yAngle);
}


//TODO rework callbacks
//Guess: Publishes a message, use pointers to access required values inside callback
//needs to read all 3 topics before implementation
void ArucoLocation::arucoCallback(const urc_msgs::msg::ArucoDetection & aruco_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received aruco!");
  xAngle = aruco_msg.x_angle;
  yAngle = aruco_msg.y_angle;
  zAngle = aruco_msg.z_angle;
  trueDist = aruco_msg.distance;
  tagId = aruco_msg.id;
  std::cout << "\n\n" << "Distance: " << trueDist << std::endl;

  if (gpsRead && orientationRead) {
    urc_msgs::msg::ArucoLocation location_message;
    location_message.header.stamp = aruco_msg.header.stamp;
    location_message.lon = 0;
    location_message.lat = 0;
    location_message.id = aruco_msg.id;
    location_publisher->publish(location_message);
  }

  orientationRead = false;
  arucoRead = false;
  gpsRead = false;
}

void ArucoLocation::gpsCallback(const sensor_msgs::msg::NavSatFix & gps_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received GPS!");
  droneLat = double(gps_msg.latitude);
  droneLon = double(gps_msg.longitude);
  droneAlt = double(gps_msg.altitude);
  gpsRead = true;
}


/*
  See http://wiki.ros.org/tf2/Tutorials/Quaternions and
  https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
*/
void ArucoLocation::orientationCallback(const sensor_msgs::msg::Imu & imu_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received orientaion!");
  roll = 0;
  pitch = 0;
  yaw = 0;
  // geometry_msgs::msg::Quaternion rawOrientation = imu_msg.orientation;
  // tf2::Quaternion convertedOrientation;
  // tf2::fromMsg(rawOrienation, convertedOrientation);
  // tf2::Matrix3x3 m(convertedOrienation);
  // m.getRPY(roll, pitch, yaw);
  //TODO convert quaternion into double values of roll, pitch, yaw
  orientationRead = true;
}

}
RCLCPP_COMPONENTS_REGISTER_NODE(aruco_location::ArucoLocation)
