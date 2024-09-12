#include <stdint.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include "my_px4_tutorial/OffboardControl.hpp"


#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// class OffboardControl : public rclcpp::Node
// {
// public:
// 	OffboardControl() : Node("offboard_control_node")
// 	{
// 		// publishers
// 		// publish the control mode : position or speed
// 		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
// 		// publish specific commands like arm or disarm
// 		vehicle_command_publishe	r_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
// 		// publish a position to reach
// 		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

// 		Takeoff();

// 	}

// 	void arm();
// 	void disarm();

// private:
// 	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
// 	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
// 	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

// 	void publish_offboard_control_mode(bool position, bool speed);
// 	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

// 	void publish_trajectory_setpoint(float x, float y, float z, float yaw);
// };

OffboardControl::OffboardControl() : Node("offboard_control_node")
{
	// publishers
	// publish the control mode : position or speed
	offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	// publish specific commands like arm or disarm
	vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
	// publish a position to reach
	trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

	// Takeoff();

}

/**
 * @brief pass to offboard control mode -> able to directly send command using PX4
 * @param position	boolean that indicates if it is controlled using position
 * @param speed		boolean that indicates if it is controlled using speed
 */
void OffboardControl::publish_offboard_control_mode(bool position, bool speed)
{
	OffboardControlMode msg{};
	msg.position = position;
	msg.velocity = speed;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/** TUTORIAL
 * @brief Publish vehicle commands -> publish specific commands like arm or disarm
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publishes a trajectory setpoint -> gives a position to reach to the drone
 * 
 * This function creates a `TrajectorySetpoint` message with the specified position coordinates and orientation angle,
 * and then publishes this message to the trajectory setpoint topic.
 * 
 * @param x 	The x position of the trajectory setpoint.
 * @param y		The y position of the trajectory setpoint.
 * @param z		The z position of the trajectory setpoint.
 * @param yaw 	The orientation angle (in radians), in the range [-PI, PI].
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}


// on se s'en servira probablement plus donc à delete
// int main(int argc, char *argv[])
// {
// 	std::cout << "Starting offboard control node" << std::endl;
// 	rclcpp::init(argc, argv);
// 	auto offboard_control_node = std::make_shared<OffboardControl>();
// 	rclcpp::spin(offboard_control_node);

// 	// rclcpp::shutdown();
// 	return 0;
// }