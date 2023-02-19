#ifndef ARUCO_BACKUP_H
#define ARUCO_BACKUP_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

namespace aruco_backup
{
class ArucoBackup : public rclcpp::Node
{
public:
  explicit ArucoBackup(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<urc_msgs::msg::ArucoLocation>::SharedPtr _cmd_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

  std::unique_ptr<diagnostic_updater::Updater> updater_ptrt
  double absoluteMaxVel, maxVel, maxVelIncr;
  int leftJoyAxis, rightJoyAxis;
  bool leftInverted, rightInverted;

  void joystick_diagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void joyCallback(const sensor_msgs::msg::Joy & msg);
};
}


#endif