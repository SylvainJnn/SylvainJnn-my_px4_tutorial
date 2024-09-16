#include <stdint.h>
#include <rclcpp/rclcpp.hpp>

// Messages
#include <px4_msgs/msg/vehicle_odometry.hpp> // TODO noter quelque part comment on en arrive la

// My includes
#include "my_px4_tutorial/OffboardControl.hpp"
#include "my_px4_tutorial/jsp.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;

jsp::jsp() : Node("jsp_node")
{
	std::cout << "DEBUT NODE  " << std::endl;

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 100), qos_profile); // originalement 5

	// // auto subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,

	vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		"/fmu/out/vehicle_odometry", 
		qos, 
		std::bind(&jsp::vehicle_odometry_callback, 
				  this,
		          std::placeholders::_1));

	setup(_controller);

	std::thread first_thread_ = std::thread(&jsp::square_hardcoded, this, std::ref(_controller)); // put the functions argument after this

	first_thread_.join(); // SANS ça ça ne peut pas

	// square_hardcoded(_controller);
    // circle(_controller);
}

// ###################
// ### Subscribers ###
// ###################

void jsp::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
	px4_msgs::msg::VehicleOdometry::SharedPtr current_msg{};
	current_msg = odom_msg;
	std::cout << "POSITION : " << std::endl;
	std::cout << "x : " << current_msg->position[0] << std::endl;
	std::cout << "y : " << current_msg->position[1] << std::endl;
	std::cout << "z : " << current_msg->position[2] << std::endl;
}

/**
 * @brief simple takeoff -> drone arms, takes off for 7 secondes and then disarms 
*/
void jsp::setup(OffboardControl& controller) // mettre const pour protéger ? 
{
	controller.publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	
	// Arm the vehicle
	controller.arm();
}


/**
 * @brief simple takeoff -> drone arms, takes off for 7 secondes and then disarms 
*/
void jsp::take_off(OffboardControl& controller)
{
	// control the drone on position
	controller.publish_offboard_control_mode(true, false); 

	auto start_time = this->get_clock()->now().nanoseconds() / 1000000;
	int flag = 1;
	while(flag)
	{
		auto current_time = this->get_clock()->now().nanoseconds() / 1000000;
		auto elapsed_time = current_time - start_time;
		if(elapsed_time < 10000) 
		{
			controller.publish_trajectory_setpoint(0.0, 0.0, -7.0, -3.14);
		}
		else
		{
			flag = 0;
			// RCLCPP_INFO(this->get_logger(), "Sutdowning");
			controller.disarm();
			// find a way to stop it 
		}
	}
}

/**
 * @brief do a sware around the global origin. 
*/
void jsp::square_hardcoded(OffboardControl& controller)
{
	RCLCPP_INFO(this->get_logger(), "INSIDE");
	int flag = 1;
	auto start_time = this->get_clock()->now().nanoseconds() / 1000000;
	int period = 5000;
	
	while(flag)
	{
		RCLCPP_INFO(this->get_logger(), "inside the loop");
		auto current_time = this->get_clock()->now().nanoseconds() / 1000000;
		auto elapsed_time = current_time - start_time;

		// control the drone on position
		controller.publish_offboard_control_mode(true, false); // must be published regulary
	

		if(elapsed_time  < 2000)
		{
			// RCLCPP_INFO(this->get_logger(), "position 0");
			controller.publish_trajectory_setpoint(0.0, 0.0, -3.0, -3.14);
		}
			
		else if(elapsed_time  < 2*period)
		{
			// RCLCPP_INFO(this->get_logger(), "position 1");
			controller.publish_trajectory_setpoint(3.0, 0.0, -6.0, -3.14);
		}
		
		else if(elapsed_time  < 3*period)
		{
			// RCLCPP_INFO(this->get_logger(), "position 2");
			controller.publish_trajectory_setpoint(0.0, 3.0, -8.0, -3.14);
		}
		
		else if(elapsed_time  < 4*period)
		{
			// RCLCPP_INFO(this->get_logger(), "position 3");
			controller.publish_trajectory_setpoint(-3.0, 0.0, -9.0, -3.14);
		}

		else if(elapsed_time  < 5*period)
		{
			// RCLCPP_INFO(this->get_logger(), "position 4");
			controller.publish_trajectory_setpoint(0.0, -3.0, -9.0, -3.14);
		}

		else if(elapsed_time  < 6*period)
		{
			// RCLCPP_INFO(this->get_logger(), "position 0 again");
			controller.publish_trajectory_setpoint(0.0, 0.0, -3.0, -3.14);
		}
		else
		{
			flag = 0;
			controller.disarm();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}

//
/**
 * @brief do circle around a position
*/


int main(int argc, char *argv[])
{
	std::cout << "Starting jsp node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<jsp>());

	// rclcpp::shutdown();
	return 0;
}


/* le dernier commit y'a pas le sqaure masi trkl, pour prochaine fois:m
 - faire un sub de odom
 - update le quare pour plus que ce soit hardcoded (on le fait autour d'odom)
*/


// COMPNRDRE PK ON reçoit rien dans el sub à certains moment
// Qos ? 
// keep_last , keep_all ? 

/*
	thread



	destructeur
	~MyNode()
    {
        if (time_monitor_thread_.joinable()) {
            time_monitor_thread_.join();  // Attendre la fin du thread
        }

*/