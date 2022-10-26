#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urc_msgs/msg/aruco_detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bits/stdc++.h>
#include <image_transport/image_transport.hpp>

namespace aruco_detector
{

class ArucoDetector : public rclcpp::Node
{
public:
    explicit ArucoDetector(const rclcpp::NodeOptions & options);

private:
    rclcpp::Publisher<urc_msgs::msg::ArucoDetection>::SharedPtr aruco_publisher; //TODO: set custom message type here
    image_transport::CameraSubscriber camera_subscriber_;

    std::vector<std::vector<cv::Point2f>> corners, rejects; // rejects will likely be unused
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
        const sensor_msgs::msg::Image & image_msg,
        const sensor_msgs::msg::CameraInfo & info_msg
    );
};
}

#endif