#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "icarus_arm_control/msg/pimu.hpp"

class ImuPimuListener : public rclcpp::Node
{
public:
    ImuPimuListener();
    void integrate();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pimu_callback(const icarus_arm_control::msg::PIMU::SharedPtr msg);

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<icarus_arm_control::msg::PIMU>::SharedPtr sub_pimu_;

    // IMU state
    double angular_velocity_x_, angular_velocity_y_, angular_velocity_z_;
    double imu_dt_;

    // Integrated angles
    double integrated_theta_ = 0.0;
    double integrated_phi_ = 0.0;
    double integrated_psi_ = 0.0;

    // Previous angles
    double prev_theta_ = 0.0;
    double prev_phi_ = 0.0;
    double prev_psi_ = 0.0;
};
