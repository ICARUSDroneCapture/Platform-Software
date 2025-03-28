/**
 * @file controller.cpp
 *
 * @brief This file contains the controller class code
 *
 */

#include "controller.hpp"

 void Controller::init()
 {
     //auto testNode = std::make_shared<Controller>();
     //testNode->init();
 
     //auto nh = std::make_shared<rclcpp::Node>("nh");
     sub_wheel_encoder_      = this->create_subscription<sensor_msgs::msg::JointState>("msg_wheel_encoder", 1, std::bind(&Controller::cbWheelEncoder, this, std::placeholders::_1));
     sub_pimu_               = this->create_subscription<icarus_arm_control::msg::PIMU>("pimu", 1, std::bind(&Controller::cbPIMU, this, std::placeholders::_1));
     sub_imu_                = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&Controller::cbIMU, this, std::placeholders::_1));
}

bool Controller::update()
{
    // static double last_time = 0;
    // if( ros::ok() )
    // {
    //     double current_time = ros::Time::now().toSec();
    //     double dt = current_time - last_time;
    //     last_time = current_time;
    //     ros::spinOnce();
    //     current_time = ros::Time::now().toSec();
    //     ros::Rate rate(50);     // 50 ms
    //     rate.sleep();
    //     dt = ros::Time::now().toSec() - current_time;

    //     return true;
    // }

    return false;
}

void Controller::cbWheelEncoder(const sensor_msgs::msg::JointState &msg)
{
    if (!quiet)
        TEST_COUT << "Rx wheel encoder : " << std::fixed << std::setw(11) << std::setprecision(6) << msg.header.stamp.sec << std::endl;
}

void Controller::cbPIMU(const icarus_arm_control::msg::PIMU::SharedPtr pimu)
{
    if (!quiet)
        TEST_COUT << "Rx PIMU : " << std::fixed << std::setw(11) << std::setprecision(6) << pimu->header.stamp.sec << std::endl;
    if (got_gps_tow)
        pimu_ts.push_back(pimu->header.stamp.sec);
    this->did_rx_pimu_ = true;
}

void Controller::cbIMU(const  sensor_msgs::msg::Imu &imu)
{
    if (!quiet)
        TEST_COUT << "Rx IMU : " << std::fixed << std::setw(11) << std::setprecision(6) << imu.header.stamp.sec << std::endl;
    if (got_gps_tow)
        imu_ts.push_back(imu.header.stamp.sec);
}

int Controller::get_deviations(std::vector<double> &a, std::vector<double> &b, std::vector<double> &out) 
{
  double total_devs = 0.0;

  out.clear();
  for (auto aa : a) {
      double min_d = 99999999.;
      // find the value from b with the nearest difference to aa
      for (auto bb : b) {
          min_d = std::min(min_d, std::abs(aa - bb));
      }
      out.push_back(min_d);
      total_devs += min_d;
  }
  return out.size();
}

double Controller::get_avg_deviation(std::vector<double> &a, std::vector<double> &b) 
{

  std::vector<double> devs;
  int num = get_deviations(a, b, devs);
  double total_devs = 0.0;

  for (auto d : devs) {
      total_devs += d;
  }
  return total_devs / (double)devs.size();
}

double Controller::get_min_deviation(std::vector<double> &a, std::vector<double> &b) 
{
  double min_d = 999999999.;

  std::vector<double> devs;
  int num = get_deviations(a, b, devs);

  for (auto d : devs) {
      min_d = std::min(min_d, d);
  }
  return min_d;
}

double Controller::get_max_deviation(std::vector<double> &a, std::vector<double> &b) 
{
  double max_d = 0.;

  std::vector<double> devs;
  int num = get_deviations(a, b, devs);

  for (auto d : devs) {
      max_d = std::max(max_d, d);
  }
  return max_d;
}