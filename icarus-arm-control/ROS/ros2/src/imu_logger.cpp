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

class ImuLoggerNode : public rclcpp::Node
{
public:
    ImuLoggerNode()
        : Node("imu_logger_node")
    {
        // Open CSV file and write headers
        file_.open("imu_data.csv", std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open imu_data.csv");
            return;
        }

        file_ << "timestamp,orientation_x,orientation_y,orientation_z,orientation_w,"
              << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
              << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z" << std::endl;

        // Create subscription to IMU topic
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&ImuLoggerNode::cbImuData, this, std::placeholders::_1));
    }

    ~ImuLoggerNode()
    {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void logToCSV(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert ROS time to system time for timestamp
        rclcpp::Time ros_time = msg->header.stamp;
        std::time_t time_sec = ros_time.seconds();
        auto fractional = ros_time.nanoseconds() % 1000000000;

        std::tm *ptm = std::localtime(&time_sec);
        std::ostringstream timestamp_stream;
        timestamp_stream << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
        timestamp_stream << "." << std::setw(3) << std::setfill('0') << (fractional / 1000000);
        std::string formatted_time = timestamp_stream.str();

        // Write IMU data to CSV
        file_ << formatted_time << ","
              << msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," << msg->orientation.w << ","
              << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
              << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z
              << std::endl;
    }

private:
    void cbImuData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        logToCSV(msg);  // Log the IMU data to the CSV
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    std::ofstream file_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImuLoggerNode>();

    rclcpp::Rate rate(10);  // 10 Hz, adjust as needed
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
