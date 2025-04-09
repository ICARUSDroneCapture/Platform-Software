/**
 * @file controller.hpp
 *
 * @brief This file contains the controller class (subclass of Node) and all member functions
 *
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

// #define DOF3_CONTROL
#define DOF1_CONTROL

// #define RK4_INTEGRATION
#define EULER_INTEGRATION

#define GRAVITY 9.81

#define GEAR_RATIO 50

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <yaml-cpp/yaml.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <array>

#include <matplot/matplot.h>

#include <chrono>
#include <memory>
#include <cassert>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include "inertial_sense_ros2/msg/gps.hpp"
// #include "inertial_sense_ros2/msg/gps_info.hpp"
// #include "inertial_sense_ros2/msg/pimu.hpp"
// #include "inertial_sense_ros2/srv/firmware_update.hpp"
// #include "inertial_sense_ros2/srv/ref_lla_update.hpp"
// #include "inertial_sense_ros2/msg/rtk_rel.hpp"
// #include "inertial_sense_ros2/msg/rtk_info.hpp"
// #include "inertial_sense_ros2/msg/gnss_ephemeris.hpp"
// #include "inertial_sense_ros2/msg/glonass_ephemeris.hpp"
// #include "inertial_sense_ros2/msg/gnss_observation.hpp"
// #include "inertial_sense_ros2/msg/gnss_obs_vec.hpp"
// #include "inertial_sense_ros2/msg/inl2_states.hpp"
// #include "inertial_sense_ros2/msg/didins2.hpp"
// #include "inertial_sense_ros2/msg/didins1.hpp"
// #include "inertial_sense_ros2/msg/didins4.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "ControlHelper.hpp"

#include <std_srvs/srv/empty.hpp>

#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"

#include "icarus_arm_control/msg/publish_data.hpp"

#include "ISUtilities.h"

 //#include "std_srvs/std_srvs/srv/emtpy.hpp"
 

 
 using std::placeholders::_1;
 using std::placeholders::_2;
 
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
    void plot(double startTime);
    void bias_calibrate();
    void imu_configure(int);
    void imu_error_correction();
    void integrate();
    void print_data();
    void remove_gravity();
    void control_testing(double iter_val);
    void control_1dof();
    void control_3dof();
    void insert_front(double *a, const int n, double val);

    void cbINS(const  inertial_sense_ros2::msg::DIDINS1::SharedPtr ins);
    void cbWheelEncoder(const sensor_msgs::msg::JointState &msg);
    void cbPIMU(const inertial_sense_ros2::msg::PIMU::SharedPtr pimu);
    void cbIMU(const sensor_msgs::msg::Imu &imu);
    void cbCtrlStatus(const  odrive_can::msg::ControllerStatus::SharedPtr ctrl_stat_);
    void cbODrvStatus(const  odrive_can::msg::ODriveStatus::SharedPtr odrv_stat_);
    void SendControlMessage(double control_torque);
    void publish_imu();

    int get_deviations(std::vector<double> &a, std::vector<double> &b, std::vector<double> &out);
    double get_avg_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_min_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_max_deviation(std::vector<double> &a, std::vector<double> &b);

    bool quiet = true;
    bool plot_quiet = true;
    bool got_gps_tow = false;
    bool did_rx_pimu_ = false;

    inertial_sense_ros2::msg::PIMU::SharedPtr pimu;
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

    //INS Values
    
    float theta_ins = 0.0;
    float phi_ins = 0.0;
    float psi_ins = 0.0;

    // Integrated angle values from IMU Values
    float integrated_theta;
    float integrated_phi;
    float integrated_psi;

    // Gravity corrected acceleration values
    float a_x_g_corrected;
    float a_y_g_corrected;
    float a_z_g_corrected;

    float theta_rg;
    float phi_rg;
    float psi_rg;

    //Bias Values
    float offset_and_turnon_bias_x;
    float offset_and_turnon_bias_y;
    float offset_and_turnon_bias_z;

    float bias_ins_theta;
    float bias_ins_phi;
    float bias_ins_psi;

    float angular_velocity_bias_u;
    float angular_velocity_bias_v;
    float angular_velocity_bias_w;
    // Motor Encoder Controller Status Values
    float encoder_position;
    float encoder_velocity;

    float motor_temperature;

    double iter_val = 0.0;

    // Message for Motor Control
    odrive_can::msg::ControlMessage msg_ctrl;
    // Message for Publish Data
    icarus_arm_control::msg::PublishData publish_data;

    // dt value retrieved from pimu
    double imu_dt;

    // ControlHelper ch_;

    #ifdef EULER_INTEGRATION
    double prev_theta;
    double prev_phi;
    double prev_psi;
    #endif

    #ifdef RK4_INTEGRATION
    // Array list for angular velocity values at 4 different timesteps
    static const int timestep_store = 4;
    
    double ang_vel_x_ts_l[timestep_store] = { };
    double ang_vel_y_ts_q[timestep_store] = { };
    double ang_vel_z_ts_q[timestep_store] = { };

    double *arr_x_ptr = ang_vel_x_ts_l;
    double *arr_y_ptr = ang_vel_y_ts_q;
    double *arr_z_ptr = ang_vel_z_ts_q; 
    #endif

    std::vector<double> gps_ts;
    std::vector<double> imu_ts;
    std::vector<double> pimu_ts;

    // Periodic plotting vectors

    std::vector<double> plot_ts = {0, 0};

    std::vector<double> plot_theta = {0, 0};
    std::vector<double> plot_phi = {0, 0};
    std::vector<double> plot_psi = {0, 0};

    std::vector<double> plot_a_x = {0, 0};
    std::vector<double> plot_a_y = {0, 0};
    std::vector<double> plot_a_z = {0, 0};

    // Error Correction: make struct/class for imu correction stuff, too tired rn lolol
    static const int data_points = 5000;

    std::array<double, data_points> a_x_misalignment = {};
    std::array<double, data_points> a_y_misalignment = {};
    std::array<double, data_points> a_z_misalignment = {};

    std::array<double, data_points> calibrate_theta = {};
    std::array<double, data_points> calibrate_phi = {};
    std::array<double, data_points> calibrate_psi = {};

    std::array<double, data_points> calibrate_u = {};
    std::array<double, data_points> calibrate_v = {};
    std::array<double, data_points> calibrate_w = {};

    // Node Start Time, set on start of main mode activity
    double nodeStartTime;

    // ----------------------------- 1DOF Control Law Related Parameters -----------------------------

    // Relative Position Control
    double kp = 1.2;  // Proportional [N/m]
    double kd = 0.4;  // Derivative [Ns/m]    
    double ki = 0;  // Integral [N/ms]
    
    // Inertial Stabilization Control
    double ka =  30;  // Acceleration Control [kg]
    double kv = 12;  // Velocity Control [kg/s]
    double ks = 0;  // Position Control [kg*s^-2]

    double pm, desired_change, angle, z_dev, pr_err, f_i, control_torque;
 
private:

    // IMU subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheel_encoder_;
    rclcpp::Subscription<inertial_sense_ros2::msg::PIMU>::SharedPtr sub_pimu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<inertial_sense_ros2::msg::DIDINS1>::SharedPtr sub_ins_;

    // Motor encoder subscriber
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_motor_cntr_stat_;
    rclcpp::Subscription<odrive_can::msg::ODriveStatus>::SharedPtr sub_odrv_stat_;
    
    //  Motor control message publisher
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_motor_cntr_msg_;
    rclcpp::Publisher<icarus_arm_control::msg::PublishData>::SharedPtr pub_imu_ins_;
 
 };
 
 
 #endif
 
