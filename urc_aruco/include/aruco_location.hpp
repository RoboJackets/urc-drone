#ifndef ARUCO_LOCATION_H
#define ARUCO_LOCATION_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "urc_msgs/msg/aruco_detection.hpp"

namespace aruco_location
{

class ArucoLocation : public rclcpp::Node
{
public:
    explicit ArucoLocation(const rclcpp::NodeOption & options);

private:

    double getNextLatitude(double d, double xAngle double yaw, double r);
    double getNextLongitude(double d, double xAngle, double yaw, double r);
    double findD(double trueD, double yAngle, double pitch);

    rclcpp::Publisher<urc_msgs::msg::ArucoLocation>::SharedPtr location_publisher;
    rclcpp::Subscription<urc_msgs::msg::ArucoDetection>::SharedPtr aruco_suscriber;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_suscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_suscriber;


    //TODO likely need type changes (sensor documentation)
    // double droneLat, droneLon;
    // double droneHeading, dronePitch;
    
    double xAngle;
    double yAngle;
    double trueDist;
    
    
    double yaw;
    double pitch;
    double roll;
    
    
    double curX;
    double curY;
    double curZ;

    void arucoCallback(const urc_msgs::msg::ArucoDetection & arucomsg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix & gpsmsg);
    void orientationCallback(const sensor_msgs::msg::Imu & imumsg);


    bool gpsRead;
    bool orientationRead;
    bool arucoRead;
}

}




#endif
