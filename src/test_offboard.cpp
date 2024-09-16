
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
        // diff between trajectory Setpoint et trajectory Waypoint ???
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

  		// move_serveur = this->create_service<std_srvs::srv::Empty>(
        //     "move",
        //     std::bind(&OffboardControl::test_pos, this, std::placeholders::_1, std::placeholders::_2)
        // );

		// me
		// velocity_publisher = this->create_publisher

		// offboard_setpoint_counter_ = 0;

		// auto timer_callback = [this]() -> void {
		// 	RCLCPP_ERROR(this->get_logger(), "JE COMPTE ");
		// 	if (offboard_setpoint_counter_ == 10) {
		// 		RCLCPP_ERROR(this->get_logger(), "I N S I D E");
		// 		// Change to Offboard mode after 10 setpoints
		// 		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

		// 		// Arm the vehicle
		// 		this->arm();
		// 	}

		// 	// offboard_control_mode needs to be paired with trajectory_setpoint
		// 	publish_offboard_control_mode();
		// 	publish_trajectory_setpoint();

		// 	// stop the counter after reaching 11
		// 	if (offboard_setpoint_counter_ < 11) {
		// 		offboard_setpoint_counter_++;
		// 	}
		// };
		// timer_ = this->create_wall_timer(100ms, timer_callback);
		if(0)
		{
			publish_offboard_control_mode(false, true); 
			publish_trajectory_setpoint(0.0, 0.0, -5.0); // parfois il faut parfois non ???? je comprenss vraiment pas pk, go retester une autre fois
			theta = 0;
			omega = 0.5;
			dt = 0.02; // timer 20ms !!!
			radius = 10;
			
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
			offboard_setpoint_counter_ = 0;
			// publish_offboard_control_mode();
			// publish_trajectory_setpoint(0.0, 0.0, -5.0);
			RCLCPP_INFO(this->get_logger(), "la on pub une fois avant, pk ???? -- SANS SHUTDOWN");
			auto timer_callback = [this]() -> void { // put that in a different fucntion
			{
				test_pos();
			//	circle();
				offboard_setpoint_counter_++;
			}


			};
			timer_ = this->create_wall_timer(20ms, timer_callback);
		}	
		else
		{
			// Takeoff0();
		}
	}

	void arm();
	void disarm();
	// void Takeoff0();


private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr move_serveur;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	// mes variables
	float theta, omega, dt;
	float radius;

	void publish_offboard_control_mode(bool position, bool speed);
	void publish_trajectory_setpoint(float x, float y, float z);
	void publish_trajectory_speed(float x, float y, float z);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void test_pos();
	//void test_pos(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
         //     	  const std::shared_ptr<std_srvs::srv::Empty::Response> response);
	void test_velocity();
	
	void circle();


};

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
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
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

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 *  J AI CHANGE
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_speed(float x, float y, float z)
{
	TrajectorySetpoint msg{};
	msg.velocity = {x, y, z};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg); // POURQUOI c'est setpoint et pas speed, je pense que j'utilise pas le bon publisher
}

/**
 * @brief Publish vehicle commands
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


void OffboardControl::test_pos()
//void OffboardControl::test_pos(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
//              				   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	// offboard_setpoint_counter_ = 0;
	// faire avec la bonne variable

	auto start_time = this->get_clock()->now().nanoseconds() / 1000000;


	while(1)
	{
		auto current_time = this->get_clock()->now().nanoseconds() / 1000000;
    	auto elapsed_time = current_time - start_time;

		if (elapsed_time % 1000 == 0)  // Tick toutes les secondes
		{
			RCLCPP_INFO(this->get_logger(), "tick");
			std::cout << "Temps écoulé : " << elapsed_time << " ms" << std::endl;
		}

		publish_offboard_control_mode(true, false);
		if(elapsed_time  < 7000)
		{
			RCLCPP_INFO(this->get_logger(), "et de 1");
			publish_trajectory_setpoint(0.0, 0.0, -7.0);
		}
		
		else if(elapsed_time  < 10000)
		{
			RCLCPP_INFO(this->get_logger(), "et de 2");
			publish_trajectory_setpoint(2.0, 1.0, -6.0);
		}
		
		else if(elapsed_time  < 15000)
		{
			RCLCPP_INFO(this->get_logger(), "et de 3");
			publish_trajectory_setpoint(2.0, 0.0, -4.0);
		}
		
		else if(elapsed_time  < 20000)
		{
			RCLCPP_INFO(this->get_logger(), "et de 4");
			publish_trajectory_setpoint(20.0, 2.0, -5.0);
		}

		else
		{

			rclcpp::shutdown();
			// this->disarm();
		}
		
		// CHATGPT // Éviter d'utiliser 100% du CPU // CHECHER LA FRÉQUENCE À LA QUELLE DONNER INSCRITION
		std::this_thread::sleep_for(std::chrono::milliseconds(300)); 
	}
}

void OffboardControl::test_velocity()
{
	publish_offboard_control_mode(false, true);
	if(offboard_setpoint_counter_ < 100)
	{
		RCLCPP_INFO(this->get_logger(), "et de 1");
		publish_trajectory_speed(0.0, 0.0, -5.0);
	}
	
	else if(offboard_setpoint_counter_ < 200)
	{
		RCLCPP_INFO(this->get_logger(), "et de speed");
		publish_trajectory_speed(1.0, 1.0, -1);
	}
	
	else if(offboard_setpoint_counter_ > 200)
	{

		rclcpp::shutdown();
		// this->disarm();
	}
	
	if((offboard_setpoint_counter_ % 10) == 0)
	{
		RCLCPP_INFO(this->get_logger(), "tick"); 
		std::cout<<offboard_setpoint_counter_<<std::endl;
	}
}


void OffboardControl::circle()
{
	publish_offboard_control_mode(true, false);
	// check nav state! 
	float new_x, new_y, new_z;

	theta = theta + omega * dt; // comprendre pk // current_pose + step * dt ? 
	new_x = radius * cos(theta); // radius 
	new_y = radius * sin(theta); 
	new_z = -5.0;
	// SPEED
	publish_trajectory_speed(new_x, 
							 new_y,
							 new_z);
}
// void jsp::circle(OffboardControl& controller) -> à faire sur la vitesse !
// {
// 	controller.publish_offboard_control_mode(true, false);
	
// 	float new_x, new_y, new_z, radius, theta, omega;

// 	radius = 5;
// 	theta = 0;
// 	omega = 0.05;
// 	rclcpp::Rate rate(20);  // 20 Hz
// 	while(1)
// 	{
// 		theta = theta + omega;
// 		new_x = radius * cos(theta); 
// 		new_y = radius * sin(theta); 
// 		new_z = -5.0;

// 		controller.publish_trajectory_setpoint(new_x,
// 											   new_y,
// 											   new_z,
// 											   -3.14);
// 	rate.sleep();
// 	}

// }



int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	// std::make_shared<OffboardControl>();
	rclcpp::spin(std::make_shared<OffboardControl>());

	// rclcpp::shutdown();
	return 0;
}



/*

les test pos sont prévu d'être testé en service

si on veut donner une lsite de position, chercherà rajouter une boucle pour savoir quand est-ce que la poistion est obetnue.
*/

