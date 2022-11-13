#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace offboard_control 
{
    class OffboardControl : public rclcpp::Node
    {
        public:
            explicit OffboardControl(const rclcpp::NodeOptions & options);
    }
}

#endif