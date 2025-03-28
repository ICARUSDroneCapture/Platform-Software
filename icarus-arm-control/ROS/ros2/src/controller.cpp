/**
 * @file controller_listener.hpp
 *
 * @brief This file contains the controller class code
 *
 */

 #include "controller.hpp"

// Controller::Controller() : rclcpp::Node()
// {
//   // Should always be enabled by default
//   // cl_.imu.grav_correction = true;
//   RCLCPP_INFO(rclcpp::get_logger("start"),"init");

// };

// void Controller::initializeROS()
// {
//   // auto topic_callback =
//   //   [this](std_msgs::msg::String::UniquePtr msg) -> void {
//   //     RCLCPP_INFO(this->get_logger(), "Sent IMU Data: '%s'", msg->data.c_str());
//   //   };
//   // subscription_ =
//   //   this->create_subscription<sensor_msgs::msg::Imu>(rs_.imu.topic, 1, topic_callback);

//   // subscription_ =
//   //   this->create_subscription<sensor_msgs::msg::Imu>(rs_.imu.topic, 1);

//   RCLCPP_INFO(rclcpp::get_logger("controller"),"Insert constructor placeholder...");
// };

// void Controller::initialize()
// {
//   RCLCPP_INFO(rclcpp::get_logger("start"),"======  STARTING CONTROLLER  ======");

//   initializeROS();
// }


// void Controller::terminate()
// {
//   RCLCPP_INFO(rclcpp::get_logger("controller"),"======  SHUTTING DOWN CONTROLLER  ======");

//   rclcpp::shutdown();
// };


// void Controller::update()
// {
//   // Calcs
//   RCLCPP_INFO(rclcpp::get_logger("controller"),"Insert calculation placeholder...");

// };

// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
