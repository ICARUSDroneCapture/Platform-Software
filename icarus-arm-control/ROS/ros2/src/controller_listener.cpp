#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// imu publisher: nh_->create_publisher<sensor_msgs::msg::Imu>(rs_.imu.topic, 1); }

class Controller : public rclcpp::Node
{
public:
  Controller()
  : Node("controller_listener_node")
  {
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Sent IMU Data: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<sensor_msgs::msg::Imu>(rs_.imu.topic, 1, topic_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Here will be the controller listener.\nThis node will do integration, gravity correction and control law calculations.\n");
  return 0;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
