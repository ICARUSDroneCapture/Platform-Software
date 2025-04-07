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
#include "icarus_arm_control/msg/controller_status.hpp"
#include "icarus_arm_control/msg/control_message.hpp"

class TorqueLoggerNode : public rclcpp::Node
{
public:
    TorqueLoggerNode()
        : Node("torque_logger_node"),
          encoder_position(0.0), encoder_velocity(0.0), commanded_torque(0.0)
    {
        // Open CSV file and write headers
        file_.open("torque_data.csv", std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open torque_data.csv");
            return;
        }

        file_ << "timestamp,commanded_torque,encoder_torque,encoder_torque_target,encoder_angle,encoder_velocity" << std::endl;

        sub_motor_cntr_stat_ = this->create_subscription<icarus_arm_control::msg::ControllerStatus>("controller_status", 1, std::bind(&TorqueLoggerNode::cbCtrlStatus, this, std::placeholders::_1));
        sub_motor_cntr_msg_ = this->create_subscription<icarus_arm_control::msg::ControlMessage>("control_message", 1, std::bind(&TorqueLoggerNode::cbCtrlMessage, this, std::placeholders::_1));
    }

    ~TorqueLoggerNode()
    {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void logToCSV()
    {
        // Get current system time for timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t time = std::chrono::system_clock::to_time_t(now);
        std::tm *ptm = std::localtime(&time);
        std::ostringstream timestamp_stream;
        timestamp_stream << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

        // Write to CSV
        file_ << timestamp_stream.str() << ","
              << commanded_torque << "," << encoder_torque << "," << encoder_torque_target << "," << encoder_position << "," << encoder_velocity
              << std::endl;
    }

    icarus_arm_control::msg::ControlMessage msg_ctrl;

private:
    void cbCtrlStatus(const icarus_arm_control::msg::ControllerStatus::SharedPtr ctrl_stat)
    {
        encoder_position = ctrl_stat->pos_estimate;
        encoder_velocity = ctrl_stat->vel_estimate;
        encoder_torque = ctrl_stat->torque_estimate;
        encoder_torque_target = ctrl_stat->torque_target;
    }

    void cbCtrlMessage(const icarus_arm_control::msg::ControlMessage::SharedPtr msg_ctrl)
    {
        commanded_torque = msg_ctrl->input_torque;
    }


    rclcpp::Subscription<icarus_arm_control::msg::ControlMessage>::SharedPtr sub_motor_cntr_msg_;
    rclcpp::Subscription<icarus_arm_control::msg::ControllerStatus>::SharedPtr sub_motor_cntr_stat_;


    std::ofstream file_;

    // Variables to store torque data
    double encoder_position;
    double encoder_velocity;
    double commanded_torque;
    double encoder_torque;
    double encoder_torque_target;




};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TorqueLoggerNode>();

    rclcpp::Rate rate(10);  // 10 Hz, adjust as needed
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->logToCSV();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
