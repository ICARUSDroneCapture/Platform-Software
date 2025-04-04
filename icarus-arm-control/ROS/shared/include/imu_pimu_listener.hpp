/**
 * @file controller.hpp
 *
 * @brief This file contains the controller class (subclass of Node) and all member functions
 *
 */

 #ifndef IMU_PIMU_LISTENER_HPP
 #define IMU_PIMU_LISTENER_HPP

class ImuPimuListener : public rclcpp::Node
{
public:
    ImuPimuListener();
    void integrate();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);
    void pimu_callback(const icarus_arm_control::msg::PIMU::SharedPtr pimu);

    // Subscriptions
    rclcpp::Subscription<icarus_arm_control::msg::PIMU>::SharedPtr sub_pimu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

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

 
#endif
