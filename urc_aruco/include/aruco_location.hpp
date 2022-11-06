#ifndef ARUCO_LOCATION_H
#define ARUCO_LOCATION_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urc_msgs/msg/aruco_detection.hpp>

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
    rclcpp::Subscription<gps::fill_me_in_later::gps>::SharedPtr gps_suscriber; //TODO
    rclcpp::Subscription<orientation::fill_me_in_later::orientation>::SharedPtr orientation_suscriber; //TODO

    double droneLat, droneLon;
    double droneHeading, dronePitch;

    void gpsCallback();
    void orientationCallback();
    void arucoCallback();
}

}




#endif