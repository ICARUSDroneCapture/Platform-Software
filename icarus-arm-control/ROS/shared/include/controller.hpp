/**
 * @file controller.hpp
 *
 * @brief This file contains the controller class and all member functions
 *
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

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
class Controller // Listener
{
public:
    Controller();

    ~Controller() { terminate(); }

    void initializeROS();
    void initialize();
    void terminate();

    void update();

    // Node handler for spin
    Node::SharedPtr nh_;
    Node::SharedPtr nh_private_;

    // struct
    // {
    //     DataHelper imu;
    //     DataHelper motor;
    // } cl_;

};


#endif

