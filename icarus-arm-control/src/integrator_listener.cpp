
#include "controller.hpp"
#include "imu_pimu.hpp"

ImuPimuListener::ImuPimuListener()
: Node("integrator_node")
{
    sub_pimu_= this->create_subscription<inertial_sense_ros2::msg::PIMU>("pimu", 1, std::bind(&ImuPimuListener::pimu_callback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&ImuPimuListener::imu_callback, this, std::placeholders::_1));

    pub_int_ = this->create_publisher<icarus_arm_control::msg::IntegratedAngles>("integrated_angles", 1);
}

void ImuPimuListener::step(){

    integrate();
    RCLCPP_INFO(this->get_logger(), "Integrated Angles: [%.4f, %.4f, %.4f]", integrated_theta_, integrated_phi_, integrated_psi_);

}



void ImuPimuListener::imu_callback(const sensor_msgs::msg::Imu &imu)
{
    angular_velocity_x_ = imu.angular_velocity.x;
    angular_velocity_y_ = imu.angular_velocity.y;
    angular_velocity_z_ = imu.angular_velocity.z;
}

void ImuPimuListener::pimu_callback(const inertial_sense_ros2::msg::PIMU::SharedPtr pimu)
{
    imu_dt_ = pimu->dt;
}

void ImuPimuListener::integrate()
{

    integrated_theta_ = prev_theta_ + (imu_dt_ * angular_velocity_x_);
    integrated_phi_   = prev_phi_   + (imu_dt_ * angular_velocity_y_);
    integrated_psi_   = prev_psi_   + (imu_dt_ * angular_velocity_z_);

    prev_theta_ = integrated_theta_;
    prev_phi_   = integrated_phi_;
    prev_psi_   = integrated_psi_;

    integrated_angles_.integrated_angles = {integrated_theta_,integrated_phi_,integrated_psi_};

    pub_int_->publish(integrated_angles_);

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPimuListener>();

    rclcpp::Rate loop_rate(250); // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->step();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


