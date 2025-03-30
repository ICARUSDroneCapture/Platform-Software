/**
 * @file controller.cpp
 *
 * @brief This file contains the controller class code
 *
 */

#include "controller.hpp"

 void Controller::init()
 {
    sub_wheel_encoder_      = this->create_subscription<sensor_msgs::msg::JointState>("msg_wheel_encoder", 1, std::bind(&Controller::cbWheelEncoder, this, std::placeholders::_1));
    sub_pimu_               = this->create_subscription<icarus_arm_control::msg::PIMU>("pimu", 1, std::bind(&Controller::cbPIMU, this, std::placeholders::_1));
    sub_imu_                = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&Controller::cbIMU, this, std::placeholders::_1));
}

void Controller::step()
{
  if (!quiet) {
    assign_data();

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\t----------------------------------\n");

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tLinear Velocity: [%f; %f; %f]\n", linear_velocity_S_x, linear_velocity_S_y, linear_velocity_S_z);
    
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tAngle: [%f; %f; %f]\n", theta, phi, psi);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tLinear Accelertion: [%f; %f; %f]\n", linear_acceleration_S_x, linear_acceleration_S_y, linear_acceleration_S_z);
    
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tAngular Velocity: [%f; %f; %f]\n", angular_velocity_x, angular_velocity_y, angular_velocity_z);
  }

  #ifdef EULER_INTEGRATION
    euler_integrate();
  #endif

  #ifdef RK4_INTEGRATION
    rk4_integrate();
  #endif

  #ifdef DOF1_CONTROL
    control_1dof();
  #endif

  #ifdef DOF3_CONTROL
    control_3dof();
  #endif
}

void Controller::plot()
{
  assign_data();

  int t_now = static_cast<int>( current_timeMs() );
  double a_S_x = static_cast<double>(linear_acceleration_S_x);
  double a_S_y = static_cast<double>(linear_acceleration_S_y);
  double a_S_z = static_cast<double>(linear_acceleration_S_z);

  auto t_p = matplot::linspace(t_now, t_now, 1);
  auto ax_p = matplot::linspace(a_S_x, a_S_x, 1);
  auto ay_p = matplot::linspace(a_S_x, a_S_y, 1);
  auto az_p = matplot::linspace(a_S_x, a_S_z, 1);

  auto ax1 = matplot::nexttile();
  auto ax2 = matplot::nexttile();
  auto ax3 = matplot::nexttile();

  auto l1 = matplot::scatter(ax1, t_p, ax_p);
  auto l2 = matplot::scatter(ax2, t_p, ay_p);
  auto l3 = matplot::scatter(ax3, t_p, az_p);

  l1->marker_face(true);
  l2->marker_face(true);
  l3->marker_face(true);

  matplot::hold({ax1, ax2, ax3}, matplot::on);

  matplot::show();
}

void Controller::debugPrint() {
  printf("\n\t\t Time [old, present]: [");
  for (int i = 0; i < timestep_store; ++i) {
    printf("%f, ", *(ts_ptr + i));
  }
  printf("]\n");
}

void Controller::assign_data()
{
  // Only store in this function... storing will be remove for efficiency

  // Store (preintegrated) delta theta values into class members
  theta = pimu->dtheta.x;
  phi = pimu->dtheta.y;
  psi = pimu->dtheta.z;

  // Store (preintegrated) delta theta values into class members
  linear_velocity_S_x = pimu->dvel.x;
  linear_velocity_S_y = pimu->dvel.y;
  linear_velocity_S_z = pimu->dvel.z;

  // Store angular velocity values into class members
  angular_velocity_x = imu.angular_velocity.x;
  angular_velocity_y = imu.angular_velocity.y;
  angular_velocity_z = imu.angular_velocity.z;

  // Store acceleration values into class members
  linear_acceleration_S_x = imu.linear_acceleration.x;
  linear_acceleration_S_y = imu.linear_acceleration.y;
  linear_acceleration_S_z = imu.linear_acceleration.z;
}

void Controller::euler_integrate()
{
  // timestep array usage: [now, now-dt, oldest + dt, oldest]

  // only the front of the array can point to data (up-to-date), rest of the array needs to be shifted with copy
  arr_x_ptr = &imu.angular_velocity.x;
  arr_y_ptr = &imu.angular_velocity.y;
  arr_z_ptr = &imu.angular_velocity.z;

  arr_x_ptr++;
  arr_y_ptr++;
  arr_z_ptr++;

  // for(int i = 1; i < timestep_store; i++) {

  //     arr_x_ptr = &imu.angular_velocity.x;

  //     // all older values must be copied
  //     ptr++;  // Move pointer to next element
  // }


//   integrated_theta = v_prev
}

void Controller::rk4_integrate()
{
  // insert 1dof control law stuff at this timestep
  printf("Controller debug placeholder.");
}

void Controller::control_1dof()
{
  // insert 1dof control law stuff at this timestep
  printf("Controller debug placeholder.");
}

void Controller::control_3dof()
{
  // insert 3dof control law stuff at this timestep
  printf("Controller debug placeholder.");
}

void Controller::insert_front(double *a, const int n, double val) {
  for (int i = n; i > 0; i--) {
     *(a+i) = *(a+i-1);
  }
  *a = val;
}

void Controller::cbWheelEncoder(const sensor_msgs::msg::JointState &msg)
{
    if (!quiet)
        std::cout << "Rx wheel encoder : " << std::fixed << std::setw(11) << std::setprecision(6) << msg.header.stamp.sec << std::endl;
}

void Controller::cbPIMU(const icarus_arm_control::msg::PIMU::SharedPtr pimu)
{
    if (!quiet)
        std::cout << "Rx PIMU : " << std::fixed << std::setw(11) << std::setprecision(6) << pimu->header.stamp.sec << std::endl;
    if (got_gps_tow)
        pimu_ts.push_back(pimu->header.stamp.sec);
    this->did_rx_pimu_ = true;

    imu_dt = pimu->dt;

    theta = pimu->dtheta.x;
    phi = pimu->dtheta.y;
    psi = pimu->dtheta.z;
    RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\tTimestamp: [%f]\n", imu_dt);
    RCLCPP_INFO(rclcpp::get_logger("data debug"),"\t\tAngle Changes: [%f; %f; %f]\n", theta, phi, psi);
    
}

void Controller::cbIMU(const  sensor_msgs::msg::Imu &imu)
{
    if (!quiet)
        std::cout << "Rx IMU : " << std::fixed << std::setw(11) << std::setprecision(6) << imu.header.stamp.sec << std::endl;
    if (got_gps_tow)
        imu_ts.push_back(imu.header.stamp.sec);
    
      // *ts_ptr = pimu->dt;
      // // // *ts_ptr = 2.0;
  
      // // // Copy the value of source_double to the memory location pointed to by destination_ptr
      // // std::memcpy(ts_ptr, &imu.header.stamp.sec, sizeof(double));
  
      // RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\tTimestamp: [%f]\n", *ts_ptr);
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