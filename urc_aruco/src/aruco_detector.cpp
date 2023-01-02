#include "aruco_detector.hpp"

namespace aruco_detector
{

ArucoDetector::ArucoDetector(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_detector", options)
{
  tagWidth = declare_parameter<int>("tagWidth");

  aruco_publisher = create_publisher<urc_msgs::msg::ArucoDetection>(
    "~/aruco",
    rclcpp::SystemDefaultsQoS()
  );
  
  //works now
  //https://github.com/ros-perception/image_common/issues/121 for imagetransport pub/subs
  
  
  camera_subscriber_ = image_transport::create_camera_subscription(
    this, "/image/front_img",
    std::bind(
      &ArucoDetector::imageCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    
   
}

void ArucoDetector::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  //delete this before PR
  //RCLCPP_INFO(this->get_logger(), "Received the image!");
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
    cv::aruco::DICT_4X4_50);

  //get image in gray scale?
  const auto cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
  cv::cvtColor(cv_image->image, cv_image->image, cv::COLOR_BGR2GRAY);

  const auto camera_matrix = cv::Mat(info_msg->k).reshape(1, 3);   //camera intrinsics
  cv::Mat distCoeffs = cv::Mat(info_msg->d);

  //Hasn't seen any tags yet
  for (int i = 0; i < numTags; ++i) {
    detectedTags[i] = 0;
  }
  
  
  //Converts the image to B&W with 4 different thresholds
  for (int i = 40; i < 220; i += 60) {
    //detects all of the tags with the current b&w threshold
    cv::aruco::detectMarkers(
      (cv_image->image > i), dictionary, corners, MarkerIDs, parameters,
      rejects);
    cv::aruco::estimatePoseSingleMarkers(corners, tagWidth/100.0, camera_matrix, distCoeffs, rvecs, tvecs);

    /*
    The below code makes some assumptions:
       First, the only tags that should be published are the tags used at the URC
       Second, only at most one of each tag should be detected
       Finally, there will be no false positives for tags ids being used at the URC
     */
     
    
    for (int id = 0; id < static_cast<int>(MarkerIDs.size()); ++id) {
      //checks if this tag has already been seen in this image and that it is a valid URC tag
      if (MarkerIDs[id] > numTags - 1 || detectedTags[MarkerIDs[id]] == 1) {
        continue;
      }
      detectedTags[MarkerIDs[id]] = 1;

      std::cout << tvecs[i] << std::endl;
      std::cout << rvecs[i] << std::endl;

      //xCenter = (corners[id][1].x + corners[id][0].x) / 2;
      //xAngle = dppx * (xCenter - cv_image->image.cols / 2);
      //xAngle = rvecs[i][0];

      //yCenter = (corners[id][2].y + corners[id][3].y) / 2;
      //yAngle = dppy * (yCenter - cv_image->image.rows / 2);
      //yAngle = rvecs[i][1];

      //width = corners[id][1].x - corners[id][0].x;
      //distance = tvecs[i][2];//(tagWidth * camera_matrix.at<uint8_t>(0, 0)) / width;      //TODO: Does this actually work??

      urc_msgs::msg::ArucoDetection aruco_message;
      aruco_message.header.stamp = info_msg->header.stamp;
      //TODO: other header messages?
      aruco_message.x_angle = rvecs[i][0];
      aruco_message.y_angle = rvecs[i][1];
      aruco_message.z_angle = rvecs[i][2];
      aruco_message.distance = tvecs[i][2];
      aruco_message.id = MarkerIDs[id];

      aruco_publisher->publish(aruco_message);
    }
    
  }
  
  
  
  
  
  
  
  
  
  
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_detector::ArucoDetector)
