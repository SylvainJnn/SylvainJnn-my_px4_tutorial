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

    // void timer_callback();
    // Publishers
	// rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	// rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	// rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    

    // // Function to publish offboard control mode
    // void publish_offboard_control_mode(bool position, bool speed);

    // // Function to publish vehicle commands (like arm or disarm)
    // void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    // // Function to publish trajectory setpoints
    // void publish_trajectory_setpoint(float x, float y, float z, float yaw);
};

#endif // JSP_HPP