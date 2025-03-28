/**
 * @file controller_listener.hpp
 *
 * @brief This file contains the controller class code
 *
 */

 #include "controller.hpp"

Controller::Controller(): cl_(rclcpp::Node::make_shared("cl_"))
{
  // Should always be enabled by default
  // cl_.imu.grav_correction = true;
  // RCLCPP_INFO(rclcpp::get_logger("start"),"init");

};

void Controller::initializeROS()
{
  // auto topic_callback =
  //   [this](std_msgs::msg::String::UniquePtr msg) -> void {
  //     RCLCPP_INFO(this->get_logger(), "Sent IMU Data: '%s'", msg->data.c_str());
  //   };
  // subscription_ =
  //   this->create_subscription<sensor_msgs::msg::Imu>(rs_.imu.topic, 1, topic_callback);
  
  auto topic_callback =
    [this](std_msgs::msg::String::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };
  subscription_ =
    this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);

  RCLCPP_INFO(rclcpp::get_logger("controller"),"Insert constructor placeholder...");
};

void Controller::initialize()
{
  RCLCPP_INFO(rclcpp::get_logger("start"),"======  STARTING CONTROLLER  ======");

  initializeROS();
}


void Controller::terminate()
{
  RCLCPP_INFO(rclcpp::get_logger("controller"),"======  SHUTTING DOWN CONTROLLER  ======");

  rclcpp::shutdown();
};


void Controller::update()
{
  // Calcs
  RCLCPP_INFO(rclcpp::get_logger("controller"),"Insert calculation placeholder...");

};
