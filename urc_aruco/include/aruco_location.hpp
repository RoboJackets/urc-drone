#ifndef ARUCO_LOCATION_H
#define ARUCO_LOCATION_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urc_msgs/msg/aruco_detection.hpp>
#include <urc_msgs/msg/aruco_location.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace aruco_location
{

class ArucoLocation : public rclcpp::Node
{
public:
    explicit ArucoLocation(const rclcpp::NodeOptions & options);

private:

    double getNextLatitude(double d, double xAngle, double yaw, double r);
    double getNextLongitude(double d, double xAngle, double yaw, double r);
    double findD(double trueD, double yAngle, double pitch);

    rclcpp::Publisher<urc_msgs::msg::ArucoLocation>::SharedPtr location_publisher;
    rclcpp::Subscription<urc_msgs::msg::ArucoDetection>::SharedPtr aruco_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscriber;


    //TODO likely need type changes (sensor documentation)
    
    double xAngle;
    double yAngle;
    double trueDist;
    
    double droneLat, droneLon;
    
    double yaw;
    double pitch;
    double roll;


    void arucoCallback(const urc_msgs::msg::ArucoDetection & arucomsg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix & gpsmsg);
    void orientationCallback(const sensor_msgs::msg::Imu & imumsg);


    bool gpsRead;
    bool orientationRead;
    bool arucoRead;
};

}




#endif
