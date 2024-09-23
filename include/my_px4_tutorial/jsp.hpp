#ifndef JSP_HPP
#define JSP_HPP

#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "my_px4_tutorial/OffboardControl.hpp"
#include "my_px4_tutorial/jsp.hpp"

#include <vector>
#include <array>


class jsp : public rclcpp::Node
{
public:
    jsp();
    ~jsp();

    void take_off(OffboardControl& controller);
    void square_hardcoded(OffboardControl& controller);
    void follow_position(OffboardControl& controller, std::vector<std::array<float,3>>& goal_poses);
    void go_back(OffboardControl& controller, std::array<float,3> _initial_pose);
    // void circle(OffboardControl& controller);

private:
    void setup(OffboardControl& controller);
    bool is_goal_reached(std::array<float,3> pose, std::array<float,3> goal, float tolerance);
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);

    OffboardControl _controller; 
    std::array<float,3> _initial_pose; // contains the inital drone position when the node starts
    std::vector<std::array<float,3>> goal_poses_; // contains all th position to be reached
    int flag_odom_sub_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
    px4_msgs::msg::VehicleOdometry::SharedPtr current_odom_msg{};

    // threads
    std::thread square_thread_;
    std::thread follow_poses_thread_;
    std::thread go_back_thread_;
};

#endif // JSP_HPP