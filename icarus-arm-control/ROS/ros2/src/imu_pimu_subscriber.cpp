#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"


class ImuPimuSubscriber : public rclcpp::Node
{
public:
    ImuPimuSubscriber()
        : Node("Imu_Pimu_Subscriber_Node")
    {

        // Subscribe to IMU
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&ImuPimuSubscriber::imu_callback, this, std::placeholders::_1));
        
        sub_pimu_ = this->create_subscription<icarus_arm_control::msg::PIMU>(
            "pimu", 10, std::bind(&ImuPimuSubscriber::pimu_callback, this, std::placeholders::_1));
    
    }


    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
    
    // Angular velocity
    double ang_x = msg->angular_velocity.x;
    double ang_y = msg->angular_velocity.y;
    double ang_z = msg->angular_velocity.z;

    // Linear acceleration
    double lin_x = msg->linear_acceleration.x;
    double lin_y = msg->linear_acceleration.y;
    double lin_z = msg->linear_acceleration.z;

    }

    void pimu_callback(const icarus_arm_control::msg::PIMU::SharedPtr msg)
    {
            // Delta theta (rotational change)
    double dtheta_x = msg->dtheta.x;
    double dtheta_y = msg->dtheta.y;
    double dtheta_z = msg->dtheta.z;

    // Delta velocity (linear velocity change)
    double dvel_x = msg->dvel.x;
    double dvel_y = msg->dvel.y;
    double dvel_z = msg->dvel.z;

    }
    double angular_velocity_x, angular_velocity_y, angular_velocity_z;
    double linear_acceleration_x, linear_acceleration_y, linear_acceleration_z;

    double dtheta_x, dtheta_y, dtheta_z;
    double dvel_x, dvel_y, dvel_z;

private: 
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<icarus_arm_control::msg::PIMU>::SharedPtr pimu_sub_;



};
