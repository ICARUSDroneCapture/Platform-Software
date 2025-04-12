#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>

#include "controller.hpp"

class LoggerNode : public rclcpp::Node
{
public:
    LoggerNode()
        : Node("logger_node")
    {
        // Open CSV file and write headers
        file_.open("logged_data.csv", std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open logged_data.csv");
            return;
        }

        file_ << "timestamp,motor_commanded_torque,motor_encoder_torque,motor_encoder_torque_target,motor_encoder_angle,motor_encoder_velocity,"
              << "imu_dt,imu_dthetax,imu_dthetay,imu_dthetaz,imu_dvelx,imu_dvely,imu_dvelz,imu_ang_velx,imu_ang_vely,imu_ang_velz,imu_lin_accelx,imu_lin_accely,imu_lin_accelz,"
              << "ins_theta,ins_phi,ins_psi"
              <<  std::endl;
        
        // Motor subscriptions
        sub_motor_cntr_stat_    = this->create_subscription<odrive_can::msg::ControllerStatus>("controller_status", 1, std::bind(&LoggerNode::cbCtrlStatusData, this, std::placeholders::_1));
        sub_motor_cntr_msg_     = this->create_subscription<odrive_can::msg::ControlMessage>("control_message", 1, std::bind(&LoggerNode::cbCtrlMessageData, this, std::placeholders::_1));

        // IMU subscriptions
        sub_pimu_               = this->create_subscription<inertial_sense_ros2::msg::PIMU>("pimu", 1, std::bind(&LoggerNode::cbPIMUData, this, std::placeholders::_1));
        sub_imu_                = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&LoggerNode::cbIMUData, this, std::placeholders::_1));
        sub_ins_                = this->create_subscription<inertial_sense_ros2::msg::DIDINS1>("ins_eul_uvw_ned", 1, std::bind(&LoggerNode::cbINSData, this, std::placeholders::_1));
    }

    ~LoggerNode()
    {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void logToCSV(double startTime)
    {
        // Get current system time for timestamp
        currTime = static_cast<double>( current_timeMs() ) - startTime;

        // Write to CSV
        file_ << currTime << ","
              << commanded_torque << "," << encoder_torque << "," << encoder_torque_target << "," << encoder_position << "," << encoder_velocity << ","
              << dt << "," << dtheta_x << "," << dtheta_y << "," << dtheta_z << "," << dvel_x << "," << dvel_y << "," << dvel_z << ","
              << ang_vel_x << "," << ang_vel_y << "," << ang_vel_z << "," << lin_accel_x << "," << lin_accel_y << "," << lin_accel_z << ","
              << theta_ins << "," << phi_ins << "," << psi_ins
              << std::endl;
    }

    odrive_can::msg::ControlMessage msg_ctrl;

    void cbCtrlStatusData(const odrive_can::msg::ControllerStatus::SharedPtr ctrl_stat)
    {
        encoder_position = ctrl_stat->pos_estimate;
        encoder_velocity = ctrl_stat->vel_estimate;
        encoder_torque = ctrl_stat->torque_estimate;
        encoder_torque_target = ctrl_stat->torque_target;
    }

    void cbCtrlMessageData(const odrive_can::msg::ControlMessage::SharedPtr msg_ctrl)
    {
        commanded_torque = msg_ctrl->input_torque;
    }

    void cbPIMUData(const inertial_sense_ros2::msg::PIMU::SharedPtr pimu)
    {
        dt = pimu->dt;

        // Store (preintegrated) delta theta values into class members
        dtheta_x = pimu->dtheta.x;
        dtheta_y = pimu->dtheta.y;
        dtheta_z = pimu->dtheta.z;

        // Store (preintegrated) delta theta values into class members
        dvel_x = pimu->dvel.x;
        dvel_y = pimu->dvel.y;
        dvel_z = pimu->dvel.z;
    }

    void cbIMUData(const sensor_msgs::msg::Imu &imu)
    {

        // Store angular velocity values into class members
        ang_vel_x = imu.angular_velocity.x;
        ang_vel_y = imu.angular_velocity.y;
        ang_vel_z = imu.angular_velocity.z;

        // Store acceleration values into class members
        lin_accel_x = imu.linear_acceleration.x;
        lin_accel_y = imu.linear_acceleration.y;
        lin_accel_z = imu.linear_acceleration.z;
    }

    void cbINSData(const inertial_sense_ros2::msg::DIDINS1::SharedPtr did_ins1)
    {
        theta_ins = did_ins1->theta[0];
        phi_ins = did_ins1->theta[1];
        psi_ins = did_ins1->theta[2];
    }

    rclcpp::Subscription<inertial_sense_ros2::msg::PIMU>::SharedPtr sub_pimu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<inertial_sense_ros2::msg::DIDINS1>::SharedPtr sub_ins_;

    rclcpp::Subscription<odrive_can::msg::ControlMessage>::SharedPtr sub_motor_cntr_msg_;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_motor_cntr_stat_;

    std::ofstream file_;

    // Timestamp variables
    double nodeStartTime;
    double currTime;

    // Variables to store encoder and torque data
    double encoder_position;
    double encoder_velocity;

    double commanded_torque;
    double encoder_torque;
    double encoder_torque_target;

    // Variables to store imu data
    double dt;

    double dtheta_x, dtheta_y, dtheta_z;
    double dvel_x, dvel_y, dvel_z;

    double ang_vel_x, ang_vel_y, ang_vel_z;
    double lin_accel_x, lin_accel_y, lin_accel_z;

    // Variables to store ins data
    double theta_ins, phi_ins, psi_ins;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LoggerNode>();

    // Node Start Time, set on start of main mode activity
    node->nodeStartTime = static_cast<double>( current_timeMs() );

    rclcpp::Rate rate(10);  // 10 Hz, adjust as needed
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->logToCSV(node->nodeStartTime);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
