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
#include <vector>
#include <array>

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

	// wait to receive a first message
	while(!flag_odom_sub_)
	{
		rclcpp::spin_some(this->get_node_base_interface());
		RCLCPP_INFO(this->get_logger(), "no sub yet");
	}

	setup(_controller);

	// std::vector<std::array<float,3>> goal_poses_;
	


	// std::thread square_thread_ = std::thread(&jsp::square_hardcoded, this, std::ref(_controller));
	// square_thread_.join();
	std::thread follow_poses_thread_ = std::thread(&jsp::follow_position, this, std::ref(_controller), std::ref(goal_poses_)); // put the functions argument after this
	follow_poses_thread_.join(); 

	go_back_thread_ = std::thread(&jsp::go_back, this, std::ref(_controller), std::ref(_initial_pose)); // put the functions argument after this
	go_back_thread_.join(); 

}

jsp::~jsp()
{
	// useless
	// if(first_thread_.joinable()) 
	// {
    //     first_thread_.join();  // destroy the thread, it requieres it to work
    // }
}

// ###################
// ### Subscribers ###
// ###################

void jsp::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
	current_odom_msg = odom_msg;
	flag_odom_sub_ = 1;
	// std::cout << "POSITION : " << std::endl;
	// std::cout << "x : " << current_odom_msg->position[0] << std::endl;
	// std::cout << "y : " << current_odom_msg->position[1] << std::endl;
	// std::cout << "z : " << current_odom_msg->position[2] << std::endl;
}

/**
 * @brief simple takeoff -> drone arms, takes off for 7 secondes and then disarms 
*/
void jsp::setup(OffboardControl& controller) // mettre const pour protéger ? 
{
	_initial_pose = {current_odom_msg->position[0],
					 current_odom_msg->position[1],
					 current_odom_msg->position[2]};

	goal_poses_.push_back({1,1,-5});
	goal_poses_.push_back({2,-2,-5});
	goal_poses_.push_back({5,3,-10});
	goal_poses_.push_back({12,-4,-8});

	controller.publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	
	// Arm the vehicle
	controller.arm();
}

bool jsp::is_goal_reached(std::array<float,3> pose, std::array<float,3> goal, float tolerance)
{
	return((std::abs(pose[0] - goal[0]) <= tolerance) &&
		   (std::abs(pose[1] - goal[1]) <= tolerance) &&
		   (std::abs(pose[2] - goal[2]) <= tolerance));
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
 * @brief do a sware around the global origin. control position
*/
void jsp::square_hardcoded(OffboardControl& controller)
{
	RCLCPP_INFO(this->get_logger(), "INSIDE");
	int square_flag = 1; // if 1 fly, otherwise, disarm

	// prediod for each pose
	int period = 5000;
	// lenght of the square
	int length = 5;

	// get the initial position pose when this function is launched from subscriber
	px4_msgs::msg::VehicleOdometry::SharedPtr initial_pose = this->current_odom_msg;
	int x0 = initial_pose->position[0];
	int y0 = initial_pose->position[1];

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
			controller.publish_trajectory_setpoint(x0, y0, -3.0, -3.14);
		}
			
		else if(elapsed_time  < 2*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 1");
			controller.publish_trajectory_setpoint(x0 + length, y0, -6.0, -3.14);
		}
		
		else if(elapsed_time  < 3*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 2");
			controller.publish_trajectory_setpoint(x0, y0 + length, -7.0, -3.14);
		}
		
		else if(elapsed_time  < 4*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 3");
			controller.publish_trajectory_setpoint(x0 - length, y0, -3.0, -3.14);
		}

		else if(elapsed_time  < 5*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 4");
			controller.publish_trajectory_setpoint(x0, y0 - length, -3.0, -3.14);
		}

		else if(elapsed_time  < 6*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 0 again");
			controller.publish_trajectory_setpoint(x0, y0, -3.0, -3.14);
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


void jsp::follow_position(OffboardControl& controller, std::vector<std::array<float,3>>& goal_poses)
{
	RCLCPP_INFO(this->get_logger(), "enter follow position");

	std::array<float,3> current_pose, current_goal;
	
	// give first goal to reach
	current_goal = goal_poses[0];
	
	while(!goal_poses.empty())
	{
		// get current position
		current_pose = {current_odom_msg->position[0],
						current_odom_msg->position[1],
						current_odom_msg->position[2]};

		// control the drone on position
		controller.publish_offboard_control_mode(true, false); // must be published regulary
		controller.publish_trajectory_setpoint(current_goal[0],
											   current_goal[1],
									    	   current_goal[2],
											   -3.14); 

		// check if the goal is reached
		if(is_goal_reached(current_pose, current_goal, 0.5))
		{   
			RCLCPP_INFO(this->get_logger(), "Goal reached");

			// delete first goal
			goal_poses.erase(goal_poses.begin());

			// update goal if there are more
			if(!goal_poses.empty())
			{
				current_goal = goal_poses[0];
				RCLCPP_INFO(this->get_logger(), "New goal set");
				std::cout << "New goal: "<< current_goal[0] << " " << current_goal[1]<< " " << current_goal[2] << std::endl;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	    rclcpp::spin_some(this->get_node_base_interface());
    }
	// change this to call go back function later
	RCLCPP_INFO(this->get_logger(), "no more goal");
	// controller.disarm();
}

/**
 * @brief go back to initial position // put this in offboard control ? + create a better move to function ? 
*/
void jsp::go_back(OffboardControl& controller, std::array<float,3> initial_position)
{
	std::array<float,3> current_pose;
	initial_position[2] = -5; // set an accesible height position.

	std::cout << initial_position[0] << " " << initial_position[1] << " " << initial_position[2] << std::endl;
	// get current position
	current_pose = {current_odom_msg->position[0],
					current_odom_msg->position[1],
					current_odom_msg->position[2]};

	while(!is_goal_reached(current_pose, initial_position, 0.5))
	{
		// get current position
		current_pose = {current_odom_msg->position[0],
						current_odom_msg->position[1],
						current_odom_msg->position[2]};

		// control the drone on position
		controller.publish_offboard_control_mode(true, false); // must be published regulary
		controller.publish_trajectory_setpoint(initial_position[0],
											   initial_position[1],
									    	   initial_position[2],
											   -3.14); 

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		rclcpp::spin_some(this->get_node_base_interface());
	}
	RCLCPP_INFO(this->get_logger(), "Back to initial position");
	controller.disarm();
}



//
/**
 * @brief do circle around a position, control in speed
*/


int main(int argc, char *argv[])
{
	std::cout << "Starting jsp node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<jsp>());
	// rclcpp::shutdown();
	return 0;
}



/*
	vecteur foloow poisiton
	change jsp to another name
	handle thread
	get initial position
	go back init position

	do a go to pose in off baord control 
	or we do it here, we change or function accordlingly 
	and we still create go back pose ? 

*/