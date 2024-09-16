#ifndef JSP_HPP
#define JSP_HPP

#include <stdint.h>
// #include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "my_px4_tutorial/OffboardControl.hpp"
#include "my_px4_tutorial/jsp.hpp"

// using namespace px4_msgs::msg;

class jsp : public rclcpp::Node
{
public:
    jsp();

    void setup(OffboardControl& controller);
    void take_off(OffboardControl& controller);
    void square_hardcoded(OffboardControl& controller);
    void circle(OffboardControl& controller);

private:
    OffboardControl _controller; 
    // int a = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    // Subscribers

    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

    std::thread first_thread_;
};

#endif // JSP_HPP