/**
 * @file controller.cpp
 *
 * @brief This file contains the controller class code
 *
 */

#include "controller.hpp"

//  #include "byte_swap.hpp"

 void Controller::init()
 {
    sub_wheel_encoder_      = this->create_subscription<sensor_msgs::msg::JointState>("msg_wheel_encoder", 1, std::bind(&Controller::cbWheelEncoder, this, std::placeholders::_1));
    sub_pimu_               = this->create_subscription<icarus_arm_control::msg::PIMU>("pimu", 1, std::bind(&Controller::cbPIMU, this, std::placeholders::_1));
    sub_imu_                = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&Controller::cbIMU, this, std::placeholders::_1));

    sub_motor_cntr_stat_    = this->create_subscription<icarus_arm_control::msg::ControllerStatus>("controller_status", 1, std::bind(&Controller::cbCtrlStatus, this, std::placeholders::_1));
    sub_odrv_stat_     = this->create_subscription<icarus_arm_control::msg::ODriveStatus>("odrive_status", 1, std::bind(&Controller::cbODrvStatus, this, std::placeholders::_1));

    pub_motor_cntr_msg_     = this->create_publisher<icarus_arm_control::msg::ControlMessage>("control_message", 1);
}

void Controller::step()
{
  if (!quiet) {
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\t----------------------------------\n");

    RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\tdt: [%f]\n", imu_dt);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tLinear Velocity: [%f; %f; %f]\n", linear_velocity_S_x, linear_velocity_S_y, linear_velocity_S_z);
    
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tAngle: [%f; %f; %f]\n", theta, phi, psi);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tLinear Acceleration: [%f; %f; %f]\n", linear_acceleration_S_x, linear_acceleration_S_y, linear_acceleration_S_z);
    
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tAngular Velocity: [%f; %f; %f]\n", angular_velocity_x, angular_velocity_y, angular_velocity_z);
  }

  // quiet = false;
    if (!quiet) {
      RCLCPP_INFO(rclcpp::get_logger("data"),"\t\t----------------------------------\n");
      
      RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tEncoder Position and Velocity: [%f; %f]\n", encoder_position, encoder_velocity);
    }
  quiet = true;

  // Correct for imu sensor errors
  imu_error_correction();

  // Integrate IMU angular velocity
  integrate();

  // Remove gravity from acceleration values
  remove_gravity();
  
  // Perform 1DOF control law
  #ifdef DOF1_CONTROL
    control_1dof();
  #endif

  #ifdef DOF3_CONTROL
    control_3dof();
  #endif

}

void Controller::plot(double startTime)
{
  double currTime = static_cast<double>( current_timeMs() ) - startTime;
  plot_ts.push_back(currTime / 1000.0);

  plot_a_x.push_back(linear_acceleration_S_x);
  plot_a_y.push_back(linear_acceleration_S_y);
  plot_a_z.push_back(linear_acceleration_S_z);

  plot_ts.erase(plot_ts.begin());

  plot_a_x.erase(plot_a_x.begin());
  plot_a_y.erase(plot_a_y.begin());
  plot_a_z.erase(plot_a_z.begin());

  auto ax1 = matplot::nexttile();
  auto ax2 = matplot::nexttile();
  auto ax3 = matplot::nexttile();

  auto l1 = matplot::scatter(ax1, plot_ts, plot_a_x);  
  auto l2 = matplot::scatter(ax2, plot_ts, plot_a_y);
  auto l3 = matplot::scatter(ax3, plot_ts, plot_a_z);

  xlim(ax1, {0, 180});
  xlim(ax2, {0, 180});
  xlim(ax3, {0, 180});

  ylim(ax1, {-2, 2});
  ylim(ax2, {-2, 2});
  ylim(ax3, {-12, -8});

  l1->marker_face(true);
  l2->marker_face(true);
  l3->marker_face(true);

  matplot::hold({ax1, ax2, ax3}, matplot::on);

  matplot::show();
  
}

void Controller::imu_configure()
{
  // !!NOTICE!! THIS IS ASSUMING IMU IS LEVEL AT START
  // CORRECT THIS ONCE WE HAVE PREDICATBLE MISSALIGNMENT VALUES

  double a_x_misalignment = linear_acceleration_S_x;
  double a_y_misalignment = linear_acceleration_S_y;
  double a_z_misalignment = linear_acceleration_S_z;
}

void Controller::imu_error_correction()
{
  linear_acceleration_S_x = linear_acceleration_S_x - a_x_misalignment;
  linear_acceleration_S_y = linear_acceleration_S_y - a_y_misalignment;
  linear_acceleration_S_z = linear_acceleration_S_z - a_z_misalignment;
}

void Controller::integrate()
{

  #ifdef EULER_INTEGRATION

  integrated_theta = prev_theta + (imu_dt * angular_velocity_x);
  integrated_phi = prev_phi + (imu_dt * angular_velocity_y);
  integrated_psi = prev_theta + (imu_dt * angular_velocity_z);

  // quiet = false;
  // if (!quiet) {
  //   RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\t----------------------------------\n");

  //   RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\tIntegrated Angle: [%f; %f; %f]\n", integrated_theta, integrated_phi, integrated_psi);
  // }
  // quiet = true;

  #endif

  #ifdef RK4_INTEGRATION
  // timestep array usage: [now, now-dt, oldest + dt, oldest]

  // // only the front of the array can point to data (up-to-date), rest of the array needs to be shifted with copy
  // arr_x_ptr = &imu.angular_velocity.x;
  // arr_y_ptr = &imu.angular_velocity.y;
  // arr_z_ptr = &imu.angular_velocity.z;

  // arr_x_ptr++;
  // arr_y_ptr++;
  // arr_z_ptr++;

  // *ts_ptr = pimu->dt;
  // // // *ts_ptr = 2.0;

  // // // Copy the value of source_double to the memory location pointed to by destination_ptr
  // // std::memcpy(ts_ptr, &imu.header.stamp.sec, sizeof(double));

  // RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\tTimestamp: [%f]\n", *ts_ptr);
  #endif
  
}

void Controller::remove_gravity()
{
  // TODO: Add check to make sure angle is getting integrated

  // angle is in radians (i think? check!!! the numbers were just smol)
  a_x_g_corrected = linear_acceleration_S_x / cos(integrated_theta);
  a_y_g_corrected = linear_acceleration_S_y / cos(integrated_phi);
  a_z_g_corrected = linear_acceleration_S_z / cos(integrated_psi) + GRAVITY;

  // quiet = false;
  if (!quiet) {
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\t----------------------------------\n");
    
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tIntegrated Angle: [%f; %f; %f]\n", integrated_theta, integrated_phi, integrated_psi);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tLinear Acceleration: [%f; %f; %f]\n", linear_acceleration_S_x, linear_acceleration_S_y, linear_acceleration_S_z);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tCorrected Acceleration: [%f; %f; %f]\n", a_x_g_corrected, a_x_g_corrected, a_x_g_corrected);
  }
  quiet = true;
}

void Controller::control_1dof()
{
  // insert 1dof control law stuff at this timestep

  double control_torque = 0.1;

  SendControlMessage(control_torque);
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

    // Store (preintegrated) delta theta values into class members
    theta = pimu->dtheta.x;
    phi = pimu->dtheta.y;
    psi = pimu->dtheta.z;

    // Store (preintegrated) delta theta values into class members
    linear_velocity_S_x = pimu->dvel.x;
    linear_velocity_S_y = pimu->dvel.y;
    linear_velocity_S_z = pimu->dvel.z;
}

void Controller::cbIMU(const  sensor_msgs::msg::Imu &imu)
{
    if (!quiet)
        std::cout << "Rx IMU : " << std::fixed << std::setw(11) << std::setprecision(6) << imu.header.stamp.sec << std::endl;
    if (got_gps_tow)
        imu_ts.push_back(imu.header.stamp.sec);
    

    // Store angular velocity values into class members
    angular_velocity_x = imu.angular_velocity.x;
    angular_velocity_y = imu.angular_velocity.y;
    angular_velocity_z = imu.angular_velocity.z;

    // Store acceleration values into class members
    linear_acceleration_S_x = imu.linear_acceleration.x;
    linear_acceleration_S_y = imu.linear_acceleration.y;
    linear_acceleration_S_z = imu.linear_acceleration.z;
}

void Controller::cbCtrlStatus(const  icarus_arm_control::msg::ControllerStatus::SharedPtr ctrl_stat_)
{
    encoder_position = ctrl_stat_->pos_estimate;
    encoder_velocity = ctrl_stat_->vel_estimate;
}

void Controller::cbODrvStatus(const  icarus_arm_control::msg::ODriveStatus::SharedPtr odrv_stat_)
{
    motor_temperature = odrv_stat_->motor_temperature;
}

void Controller::SendControlMessage(double control_torque)
{
  msg_ctrl.input_torque = control_torque;
  pub_motor_cntr_msg_->publish(msg_ctrl);
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