/**
 * @file controller.hpp
 *
 * @brief This file contains the controller class (subclass of Node) and all member functions
 *
 */

#ifndef IMU_PIMU_LISTENER_HPP
#define IMU_PIMU_LISTENER_HPP
#include "sensor_msgs/msg/imu.hpp"  // Include the IMU message header
#include "inertial_sense_ros2/msg/pimu.hpp"
#include "icarus_arm_control/msg/integrated_angles.hpp"

class ImuPimuListener : public rclcpp::Node
{
public:
    ImuPimuListener();
    void integrate();
    void step();

    //const icarus_arm_control::msg::PIMU pimu;
    //const sensor_msgs::msg::Imu::SharedPtr imu

    icarus_arm_control::msg::IntegratedAngles integrated_angles_;

private:
    void imu_callback(const sensor_msgs::msg::Imu &imu);
    void pimu_callback(const inertial_sense_ros2::msg::PIMU::SharedPtr pimu);
    
    // Subscriptions
    rclcpp::Subscription<inertial_sense_ros2::msg::PIMU>::SharedPtr sub_pimu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    //Publishers
    rclcpp::Publisher<icarus_arm_control::msg::IntegratedAngles>::SharedPtr pub_int_;

    // IMU state
    double angular_velocity_x_, angular_velocity_y_, angular_velocity_z_;
    double imu_dt_;

    // Integrated angles
    double integrated_theta_ = 0.0;
    double integrated_phi_ = 0.0;
    double integrated_psi_ = 0.0;

    //Integrated Angles array
    

    // Previous angles
    double prev_theta_ = 0.0;
    double prev_phi_ = 0.0;
    double prev_psi_ = 0.0;
};

 
#endif