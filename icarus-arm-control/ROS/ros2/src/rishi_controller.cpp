#include "controller.hpp"
#include <fstream>
#include <vector>
#include <numeric>

void Controller::init()
{
    sub_wheel_encoder_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "msg_wheel_encoder", 1, std::bind(&Controller::cbWheelEncoder, this, std::placeholders::_1));
    sub_pimu_ = this->create_subscription<icarus_arm_control::msg::PIMU>(
        "pimu", 1, std::bind(&Controller::cbPIMU, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 1, std::bind(&Controller::cbIMU, this, std::placeholders::_1));

    imu_log_file_.open("imu_data.csv");
    imu_log_file_ << "timestamp,linear_acceleration_z\n";
}

void Controller::cbIMU(const sensor_msgs::msg::Imu &imu)
{
    if (!quiet)
        std::cout << "Rx IMU : " << std::fixed << std::setw(11) << std::setprecision(6) << imu.header.stamp.sec << std::endl;

    if (got_gps_tow)
        imu_ts.push_back(imu.header.stamp.sec);

    // Store angular velocity values
    angular_velocity_x = imu.angular_velocity.x;
    angular_velocity_y = imu.angular_velocity.y;
    angular_velocity_z = imu.angular_velocity.z;

    // Store acceleration values
    linear_acceleration_S_x = imu.linear_acceleration.x;
    linear_acceleration_S_y = imu.linear_acceleration.y;
    linear_acceleration_S_z = imu.linear_acceleration.z;

    // Log linear_acceleration_z to CSV
    rclcpp::Time ros_time = imu.header.stamp;
    imu_log_file_ << ros_time.seconds() << "," << linear_acceleration_S_z << "\n";

    // Store for bias calculation
    imu_z_data_.push_back(linear_acceleration_S_z);
}

Controller::~Controller()
{
    if (imu_log_file_.is_open()) {
        imu_log_file_.close();
    }
    compute_imu_bias();
}

void Controller::compute_imu_bias()
{
    if (!imu_z_data_.empty()) {
        double sum = std::accumulate(imu_z_data_.begin(), imu_z_data_.end(), 0.0);
        double avg = sum / imu_z_data_.size();
        imu_bias_z = avg + GRAVITY;
        RCLCPP_INFO(rclcpp::get_logger("bias"), "Computed IMU bias (Z): %f", imu_bias_z);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("bias"), "No IMU data to compute bias.");
    }
}
