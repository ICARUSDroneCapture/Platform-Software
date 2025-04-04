#include "controller.hpp"
#include "imu_pimu_listener.hpp"

ImuPimuListener::ImuPimuListener()
: Node("imu_pimu_listener_node")
{
    sub_pimu_               = this->create_subscription<icarus_arm_control::msg::PIMU>("pimu", 1, std::bind(&ImuPimuListener::imu_callback, this, std::placeholders::_1));
    sub_imu_                = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&ImuPimuListener::pimu_callback, this, std::placeholders::_1));
    
}

void ImuPimuListener::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    angular_velocity_x_ = msg->angular_velocity.x;
    angular_velocity_y_ = msg->angular_velocity.y;
    angular_velocity_z_ = msg->angular_velocity.z;
}

void ImuPimuListener::pimu_callback(const icarus_arm_control::msg::PIMU::SharedPtr pimu)
{
    imu_dt_ = msg->dt;
}

void ImuPimuListener::integrate()
{
    integrated_theta_ = prev_theta_ + (imu_dt_ * angular_velocity_x_);
    integrated_phi_   = prev_phi_   + (imu_dt_ * angular_velocity_y_);
    integrated_psi_   = prev_psi_   + (imu_dt_ * angular_velocity_z_);

    prev_theta_ = integrated_theta_;
    prev_phi_   = integrated_phi_;
    prev_psi_   = integrated_psi_;

    RCLCPP_INFO(this->get_logger(), "Integrated Angles: [%.4f, %.4f, %.4f]", integrated_theta_, integrated_phi_, integrated_psi_);
}
