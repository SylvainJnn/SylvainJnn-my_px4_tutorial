#ifndef JSP_HPP
#define JSP_HPP

#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "my_px4_tutorial/OffboardControl.hpp"
#include "my_px4_tutorial/jsp.hpp"

// using namespace px4_msgs::msg;
#include <vector>
#include <array>


class jsp : public rclcpp::Node
{
public:
    jsp();
    ~jsp();

    void setup(OffboardControl& controller);
    void take_off(OffboardControl& controller);
    void square_hardcoded(OffboardControl& controller);
    void follow_position(OffboardControl& controller, std::vector<std::array<float,3>>& goal_poses);
    // void go_back(OffboardControl& controller)
    // void circle(OffboardControl& controller);

private:
    OffboardControl _controller; 
    // int a = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    // Subscribers

    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

    px4_msgs::msg::VehicleOdometry::SharedPtr current_odom_msg{};

    std::thread first_thread_;
    std::thread th_2;

    int flag_odom_sub_;
};

#endif // JSP_HPP