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

using namespace std::chrono_literals;

class ImuLoggerNode : public rclcpp::Node
{
public:
    ImuLoggerNode()
        : Node("imu_logger_node")
    {
        file_.open("imu_data.csv", std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open imu_data.csv");
            return;
        }

        // CSV header
        file_ << "timestamp,orientation_x,orientation_y,orientation_z,orientation_w,"
              << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
              << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z" << std::endl;

        // Subscribe to IMU
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10,
            std::bind(&ImuLoggerNode::topic_callback, this, std::placeholders::_1));

        // Timer to flush buffer every 5 seconds
        flush_timer_ = this->create_wall_timer(
            5s, std::bind(&ImuLoggerNode::flush_buffer_to_csv, this));
    }

    ~ImuLoggerNode()
    {
        flush_buffer_to_csv(); // Final flush
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time ros_time = msg->header.stamp;
        std::time_t time_sec = ros_time.seconds();
        auto fractional = ros_time.nanoseconds() % 1000000000;

        std::tm *ptm = std::localtime(&time_sec);
        std::ostringstream timestamp_stream;
        timestamp_stream << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
        timestamp_stream << "." << std::setw(3) << std::setfill('0') << (fractional / 1000000);
        std::string formatted_time = timestamp_stream.str();

        std::ostringstream row;
        row << formatted_time << ","
            << msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," << msg->orientation.w << ","
            << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
            << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z;

        buffer_.push_back(row.str());
    }

    void flush_buffer_to_csv()
    {
        if (!buffer_.empty()) {
            for (const auto &line : buffer_) {
                file_ << line << std::endl;
            }
            file_.flush();
            buffer_.clear();
            RCLCPP_INFO(this->get_logger(), "Flushed IMU buffer to CSV.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr flush_timer_;
    std::ofstream file_;
    std::vector<std::string> buffer_;
};
