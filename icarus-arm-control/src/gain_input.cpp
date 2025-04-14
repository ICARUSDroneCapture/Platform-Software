#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class GainInput : public rclcpp::Node
{
public:
  GainInput()
  : Node("gain_input"),
    kp_(0.0), kd_(0.0), ki_(0.0),
    ka_(0.0), kv_(0.0), ks_(0.0)
  {
    // Declare dynamic parameters for gains
    this->declare_parameter<double>("kp", kp_);
    this->declare_parameter<double>("kd", kd_);
    this->declare_parameter<double>("ki", ki_);

    this->declare_parameter<double>("ka", ka_);
    this->declare_parameter<double>("kv", kv_);
    this->declare_parameter<double>("ks", ks_);

    // Set a parameter callback so that whenever parameters change,
    // the internal variables are updated live.
    auto param_callback = [this](const std::vector<rclcpp::Parameter>& params)
    -> rcl_interfaces::msg::SetParametersResult {
    for (const auto & param : params) {
      if (param.get_name() == "kp") {
        kp_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated kp to: %f", kp_);
      } else if (param.get_name() == "kd") {
        kd_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated kd to: %f", kd_);
      } else if (param.get_name() == "ki") {
        ki_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated ki to: %f", ki_);
      } else if (param.get_name() == "ka") {
        ka_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated ka to: %f", ka_);
      } else if (param.get_name() == "kv") {
        kv_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated kv to: %f", kv_);
      } else if (param.get_name() == "ks") {
        ks_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated ks to: %f", ks_);
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Gains updated successfully";
    return result;
  };
  
    this->add_on_set_parameters_callback(param_callback);

    // Create a publisher to send the current gains
    gain_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gain_topic", 10);

    timer_ = this->create_wall_timer(100ms, [this]() {
        // Optionally refresh the parameters
    this->get_parameter("kp", kp_);
    this->get_parameter("ka", ka_);
    this->get_parameter("ki", ki_);
    this->get_parameter("kd", kd_);
    this->get_parameter("kv", kv_);
    this->get_parameter("ks", ks_);

      std_msgs::msg::Float64MultiArray msg;
      // Order: kp, ka, ki, kd, kv, ks
      msg.data = { kp_, ka_, ki_, kd_, kv_, ks_ };
      gain_pub_->publish(msg);
    });
    
  }

private:
  // Internal storage for gains
  double kp_,ka_,ki_,kd_,kv_,ks_;

  // Publisher and timer for publishing the gains
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gain_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GainInput>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
