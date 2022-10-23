#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bits/stdc++.h>

namespace aruco_detector
{

class ArucoDetector : public rclcpp::Node
{
public:
    explicit ArucoDetector(const rclcpp::NodeOptions & options);

private:
    rclcpp::Publisher aurco_publisher; //TODO: set custom message type here
    image_transport::CameraSubscriber camera_subscriber_;

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> MarkerIDs;

    float tagWidth; //actual tag width in cm
    float width;
    float xCenter, yCenter;
    float distance;
    float xAngle, yAngle;

    int detectedTags[6];

    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg
    );
};
}

#endif