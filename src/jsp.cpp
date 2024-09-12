#include <stdint.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include "my_tests/OffboardControl.hpp"
#include "my_tests/jsp.hpp"


#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// class jsp : public rclcpp::Node
// {
// public:
// 	jsp() : Node("jsp")
// 	{
//         Takeoff();
//     }


// private:
//     void Takeoff();
// 	// variable offboartd control
// };

jsp::jsp() : Node("jsp_node")
{
    //Takeoff();
}

/**
 * @brief simple takeoff -> drone arms, takes off for 7 secondes and then disarms 
*/
void jsp::Takeoff()
{
    OffboardControl my_offboard;
    my_offboard.publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);


	// Arm the vehicle
	// this->arm(); // when in the same node 
	// my_offboard.arm();// tester avec et sans!! 
	my_offboard.publish_offboard_control_mode(false, true); 

	auto start_time = this->get_clock()->now().nanoseconds() / 1000000;
	int flag = 1;
	while(flag)
	{
		auto current_time = this->get_clock()->now().nanoseconds() / 1000000;
		auto elapsed_time = current_time - start_time;
		if(elapsed_time < 10000) 
		{
			my_offboard.publish_trajectory_setpoint(0.0, 0.0, -7.0, -3.14);
		}
		else
		{
			flag = 0;
			// RCLCPP_INFO(this->get_logger(), "Sutdowning");
			my_offboard.disarm();
			// find a way to stop it 
		}

		//std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
	}
}


int main(int argc, char *argv[])
{
	std::cout << "Starting jsp node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<jsp>());

	// rclcpp::shutdown();
	return 0;
}