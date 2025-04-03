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
        : Node("imu_logger_node"), z_bias_(0.0)
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

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 1,
            std::bind(&ImuLoggerNode::topic_callback, this, std::placeholders::_1));
        // Subscribe to IMU topic
        //subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //    "/imu/data", 10,
        //    std::bind(&ImuLoggerNode::topic_callback, this, std::placeholders::_1));

            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&ImuLoggerNode::topic_callback, this, std::placeholders::_1));
    }

    ~ImuLoggerNode()
    {
        if (file_.is_open()) {
            file_.close();
        }

        compute_z_bias_from_csv("imu_data.csv");
        RCLCPP_INFO(this->get_logger(), "Z-axis bias (gravity-compensated): %.6f", z_bias_);
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert ROS time to system time
        rclcpp::Time ros_time = msg->header.stamp;
        std::time_t time_sec = ros_time.seconds();
        auto fractional = ros_time.nanoseconds() % 1000000000;

        std::tm *ptm = std::localtime(&time_sec);
        std::ostringstream timestamp_stream;
        timestamp_stream << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
        timestamp_stream << "." << std::setw(3) << std::setfill('0') << (fractional / 1000000);
        std::string formatted_time = timestamp_stream.str();

        // Write to CSV
        file_ << formatted_time << ","
              << msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," << msg->orientation.w << ","
              << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
              << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z
              << std::endl;
    }

    void compute_z_bias_from_csv(const std::string &filename)
    {
        std::ifstream infile(filename);
        std::string line;
        std::vector<double> z_values;
        const double GRAVITY = 9.80665;

        // Skip header
        std::getline(infile, line);

        while (std::getline(infile, line)) {
            std::stringstream ss(line);
            std::string token;
            int column = 0;
            double value;
            while (std::getline(ss, token, ',')) {
                if (column == 10) {  // linear_acceleration_z is the 11th column (index 10)
                    try {
                        value = std::stod(token);
                        z_values.push_back(value);
                    } catch (...) {
                        continue;
                    }
                    break;
                }
                ++column;
            }
        }

        if (!z_values.empty()) {
            double sum = 0.0;
            for (double z : z_values) {
                sum += z;
            }
            double average = sum / z_values.size();
            z_bias_ = average - GRAVITY;
        } else {
            RCLCPP_WARN(this->get_logger(), "No Z-axis acceleration data found to compute bias.");
            z_bias_ = 0.0;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    std::ofstream file_;
    double z_bias_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
