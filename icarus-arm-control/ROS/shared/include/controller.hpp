controller.hpp


/**
 * @file controller.hpp
 *
 * @brief This file contains the controller class and all member functions
 *
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace rclcpp;

/**
 * Implementation of ICARUS arm stabilizing control law
 *
 * Listens to imu publisher and odrive motor publisher.
 * Calls services to integrate readings as needed, perform gravity correction, and do control law calculations.
 * Sends stabilizing commands to motors.
 *
 */
// class Controller : public rclcpp::Node // Listener
// {
// public:
//     Controller();
//     ~Controller() { terminate(); }

//     void initializeROS();
//     void initialize();
//     void terminate();

// private:

//     void update();

//     // struct
//     // {
//     //     DataHelper imu;
//     //     DataHelper motor;
//     // } cl_;

// };





// class Controller : public rclcpp::Node
// {
// public:
//     Controller(const std::string& node_name);
// private:
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };


#endif // CONTROLLER_HPP

