/**
 * @file controller.hpp
 *
 * @brief This file contains the controller class (subclass of Node) and all member functions
 *
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <yaml-cpp/yaml.h>
#include <termios.h>
#include <unistd.h>

#include <matplot/matplot.h>

#include "data_sets.h"
#include "InertialSense.h"

#include "TopicHelper.h"

#include <chrono>
#include <memory>
#include <cassert>
#include "rclcpp/rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp/timer.hpp"
#include "rclcpp/rclcpp/time.hpp"
#include "rclcpp/rclcpp/publisher.hpp"
#include "std_msgs/std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "icarus_arm_control/msg/gps.hpp"
#include "icarus_arm_control/msg/gps_info.hpp"
#include "icarus_arm_control/msg/pimu.hpp"
#include "icarus_arm_control/srv/firmware_update.hpp"
#include "icarus_arm_control/srv/ref_lla_update.hpp"
#include "icarus_arm_control/msg/rtk_rel.hpp"
#include "icarus_arm_control/msg/rtk_info.hpp"
#include "icarus_arm_control/msg/gnss_ephemeris.hpp"
#include "icarus_arm_control/msg/glonass_ephemeris.hpp"
#include "icarus_arm_control/msg/gnss_observation.hpp"
#include "icarus_arm_control/msg/gnss_obs_vec.hpp"
#include "icarus_arm_control/msg/inl2_states.hpp"
#include "icarus_arm_control/msg/didins2.hpp"
#include "icarus_arm_control/msg/didins1.hpp"
#include "icarus_arm_control/msg/didins4.hpp"
#include "nav_msgs/nav_msgs/msg/odometry.hpp"
#include "std_srvs/std_srvs/srv/trigger.hpp"
#include "std_msgs/std_msgs/msg/header.hpp"
#include "geometry_msgs/geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "diagnostic_msgs/diagnostic_msgs/msg/diagnostic_array.hpp"

using namespace rclcpp;
using namespace icarus_arm_control;
using namespace std::chrono_literals;
 
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
    Controller() : Node("controller_listener_node") {
        // auto raw_data_f = matplot::figure()
        matplot::tiledlayout(3, 1);
    }
    void init();
    void step();
    void plot();

    void assign_data();

    void cbWheelEncoder(const sensor_msgs::msg::JointState &msg);
    void cbPIMU(const icarus_arm_control::msg::PIMU::SharedPtr pimu);
    void cbIMU(const sensor_msgs::msg::Imu &imu);

    int get_deviations(std::vector<double> &a, std::vector<double> &b, std::vector<double> &out);
    double get_avg_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_min_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_max_deviation(std::vector<double> &a, std::vector<double> &b);

    bool quiet = true;
    bool plot_quiet = true;
    bool got_gps_tow = false;
    bool did_rx_pimu_ = false;

    icarus_arm_control::msg::PIMU::SharedPtr pimu;
    sensor_msgs::msg::Imu imu;

    // IMU Pre-integrated Values
    float linear_velocity_S_x = 0.0;
    float linear_velocity_S_y = 0.0;
    float linear_velocity_S_z = 0.0;

    float theta = 0.0;
    float phi = 0.0;
    float psi = 0.0;

    // IMU Values
    float linear_acceleration_S_x = 0.0;
    float linear_acceleration_S_y = 0.0;
    float linear_acceleration_S_z = 0.0;

    float angular_velocity_x = 0.0;
    float angular_velocity_y = 0.0;
    float angular_velocity_z = 0.0;

    // Array list for angular velocity values at 4 different timesteps
    static const int timestep_store = 4;

    float ang_vel_x_ts_l[timestep_store] = { };
    float ang_vel_y_ts_q[timestep_store] = { };
    float ang_vel_z_ts_q[timestep_store] = { };

    std::vector<double> gps_ts;
    std::vector<double> imu_ts;
    std::vector<double> pimu_ts;
 
private:

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheel_encoder_;
    rclcpp::Subscription<icarus_arm_control::msg::PIMU>::SharedPtr sub_pimu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
 
 };
 
 
 #endif
 
