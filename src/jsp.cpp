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
	flag_odom_sub_ = 0;

	// Setup QoS and subscriber
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile); // originalement 5

	vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		"/fmu/out/vehicle_odometry", 
		qos, 
		std::bind(&jsp::vehicle_odometry_callback, 
				  this,
		          std::placeholders::_1));

	while(!flag_odom_sub_)
	{
		rclcpp::spin_some(this->get_node_base_interface());
		std::cout << "No sub yet" <<std::endl;
	}
	
	setup(_controller);
	std::thread first_thread_ = std::thread(&jsp::square_2, this, std::ref(_controller)); // put the functions argument after this
	first_thread_.join(); 
}

jsp::~jsp()
{
	if(first_thread_.joinable()) 
	{
        first_thread_.join();  // destroy the thread, it requieres it to work
    }
}

// ###################
// ### Subscribers ###
// ###################

void jsp::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
	current_odom_msg = odom_msg;
	flag_odom_sub_ = 1;
	std::cout << "POSITION : " << std::endl;
	std::cout << "x : " << current_odom_msg->position[0] << std::endl;
	std::cout << "y : " << current_odom_msg->position[1] << std::endl;
	std::cout << "z : " << current_odom_msg->position[2] << std::endl;
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
			RCLCPP_INFO(this->get_logger(), "Sutdowning");
			controller.disarm();
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
			RCLCPP_INFO(this->get_logger(), "position 0");
			controller.publish_trajectory_setpoint(0.0, 0.0, -3.0, -3.14);
		}
			
		else if(elapsed_time  < 2*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 1");
			controller.publish_trajectory_setpoint(3.0, 0.0, -6.0, -3.14);
		}
		
		else if(elapsed_time  < 3*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 2");
			controller.publish_trajectory_setpoint(0.0, 3.0, -8.0, -3.14);
		}
		
		else if(elapsed_time  < 4*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 3");
			controller.publish_trajectory_setpoint(-3.0, 0.0, -9.0, -3.14);
		}

		else if(elapsed_time  < 5*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 4");
			controller.publish_trajectory_setpoint(0.0, -3.0, -9.0, -3.14);
		}

		else if(elapsed_time  < 6*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 0 again");
			controller.publish_trajectory_setpoint(0.0, 0.0, -3.0, -3.14);
		}
		else
		{
			flag = 0;
			controller.disarm();
		}
		rclcpp::spin_some(this->get_node_base_interface()); // ça marche mais ce serait bien de trouver une autre option
	}
}

void jsp::square_2(OffboardControl& controller)
{
	RCLCPP_INFO(this->get_logger(), "INSIDE");
	int square_flag = 1; // if 1 fly, otherwise, disarm

	// prediod for each pose
	int period = 5000;
	// lenght of the square
	int length = 5;

	// get initial pose from subscriber
	px4_msgs::msg::VehicleOdometry::SharedPtr initial_pose = this->current_odom_msg;
	int x = initial_pose->position[0];
	int y = initial_pose->position[1];

	auto start_time = this->get_clock()->now().nanoseconds() / 1000000;

	while(square_flag)
	{
		auto current_time = this->get_clock()->now().nanoseconds() / 1000000;
		auto elapsed_time = current_time - start_time;

		// control the drone on position
		controller.publish_offboard_control_mode(true, false); // must be published regulary


		if(elapsed_time  < 2000)
		{
			RCLCPP_INFO(this->get_logger(), "position 0");
			controller.publish_trajectory_setpoint(x, y, -3.0, -3.14);
		}
			
		else if(elapsed_time  < 2*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 1");
			controller.publish_trajectory_setpoint(x + length, y, -6.0, -3.14);
		}
		
		else if(elapsed_time  < 3*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 2");
			controller.publish_trajectory_setpoint(x, y + length, -7.0, -3.14);
		}
		
		else if(elapsed_time  < 4*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 3");
			controller.publish_trajectory_setpoint(x - length, y, -3.0, -3.14);
		}

		else if(elapsed_time  < 5*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 4");
			controller.publish_trajectory_setpoint(x, y - length, -3.0, -3.14);
		}

		else if(elapsed_time  < 6*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 0 again");
			controller.publish_trajectory_setpoint(x, y, -3.0, -3.14);
		}
		else
		{
			square_flag = 0;
			controller.disarm();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		rclcpp::spin_some(this->get_node_base_interface()); // ça marche mais ce serait bien de trouver une autre option
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