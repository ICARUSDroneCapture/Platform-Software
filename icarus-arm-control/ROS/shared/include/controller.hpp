/**
 * @file controller.hpp
 *
 * @brief This file contains the controller class (subclass of Node) and all member functions
 *
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H
 
#include <memory>
#include <termios.h>
#include <unistd.h>

#include "inertial_sense_ros.h"
// #include "gtest_helpers.h"
 
 /**
  * Implementation of ICARUS arm stabilizing control law
  *
  * Listens to imu publisher and odrive motor publisher.
  * Calls services to integrate readings as needed, perform gravity correction, and do control law calculations.
  * Sends stabilizing commands to motors.
  *
  */
 class Controller : public rclcpp::Node // Listener
 {
 public:
    Controller() : Node("controller_listener_node"){}
    void init();
    void update();
    void cbWheelEncoder(const sensor_msgs::msg::JointState &msg);
    void cbPIMU(const icarus_arm_control::msg::PIMU::SharedPtr pimu);
    void cbIMU(const  sensor_msgs::msg::Imu &imu);

    int get_deviations(std::vector<double> &a, std::vector<double> &b, std::vector<double> &out);
    double get_avg_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_min_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_max_deviation(std::vector<double> &a, std::vector<double> &b);

    bool quiet = true;
    bool got_gps_tow = false;
    bool did_rx_pimu_ = false;

    std::vector<double> gps_ts;
    std::vector<double> imu_ts;
    std::vector<double> pimu_ts;
    std::vector<double> ins_ts;

    std::vector<double> gps_ts;
    std::vector<double> imu_ts;
    std::vector<double> pimu_ts;
    std::vector<double> ins_ts;
 
private:

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheel_encoder_;
    rclcpp::Subscription<icarus_arm_control::msg::PIMU>::SharedPtr sub_pimu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
 
 };
 
 
 #endif
 