#ifndef OFFBOARDCONTROL_HPP
#define OFFBOARDCONTROL_HPP

// ? 
#include <stdint.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl();
    ~OffboardControl();
    void arm();
    void disarm();
    
    // Function to publish offboard control mode
    void publish_offboard_control_mode(bool position, bool speed);

    // Function to publish vehicle commands (like arm or disarm)
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    // Function to publish trajectory setpoints
    void publish_trajectory_setpoint(float x, float y, float z, float yaw);


private:
    // Publishers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

};

#endif // OFFBOARDCONTROL_HPP