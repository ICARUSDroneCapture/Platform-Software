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
    sub_ins_                = this->create_subscription<icarus_arm_control::msg::DIDINS1>("ins_eul_uvw_ned", 1, std::bind(&Controller::cbINS, this, std::placeholders::_1));

    sub_motor_cntr_stat_    = this->create_subscription<icarus_arm_control::msg::ControllerStatus>("controller_status", 1, std::bind(&Controller::cbCtrlStatus, this, std::placeholders::_1));
    sub_odrv_stat_     = this->create_subscription<icarus_arm_control::msg::ODriveStatus>("odrive_status", 1, std::bind(&Controller::cbODrvStatus, this, std::placeholders::_1));

    msg_ctrl.control_mode = 1;
    msg_ctrl.input_mode = 1;
    pub_motor_cntr_msg_     = this->create_publisher<icarus_arm_control::msg::ControlMessage>("control_message", 1);
    pub_imu_ins_            = this->create_publisher<icarus_arm_control::msg::PublishData>("publish_data", 1);

    
}

void Controller::step()
{



  // Correct for imu sensor errors
  imu_error_correction();

  // Integrate IMU angular velocity
  //integrate();

  // Remove gravity from acceleration values
  remove_gravity();
  
  // Perform 1DOF control law

  iter_val = iter_val + 0.01;
  control_testing(iter_val);
  // control_1dof();

  //Print
   // print_data();



  publish_imu();

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

void Controller::print_data(){


  quiet = true;
  if (!quiet) {
    RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\t----------------------------------\n");

    RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\tIntegrated Angle: [%f; %f; %f]\n", integrated_theta, integrated_phi, integrated_psi);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tINS Corrected: [%f; %f; %f]\n", theta_ins, phi_ins, psi_ins);

  }
  quiet = false;
  if (!quiet) {
    RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\t----------------------------------\n");

    RCLCPP_INFO(rclcpp::get_logger("debug"),"\t\tAccelerometer Data: [%f; %f; %f]\n", linear_acceleration_S_x,linear_acceleration_S_y,linear_acceleration_S_z);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tINS Corrected: [%f; %f; %f]\n", theta_ins, phi_ins, psi_ins);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tGravity Corrected: [%f; %f; %f]\n", a_x_g_corrected,a_y_g_corrected,a_z_g_corrected);

  }


}

void Controller::imu_configure(int index)
{
  // !!NOTICE!! THIS IS ASSUMING IMU IS LEVEL AT START
  // CORRECT THIS ONCE WE HAVE PREDICATBLE MISSALIGNMENT VALUES

   a_x_misalignment[index] = linear_acceleration_S_x;
   a_y_misalignment[index] = linear_acceleration_S_y;
   a_z_misalignment[index] = linear_acceleration_S_z + GRAVITY;

  calibrate_theta[index] = theta_ins;
  calibrate_psi[index] = psi_ins;
  calibrate_phi[index] = phi_ins;

  calibrate_u[index] = angular_velocity_x;
  calibrate_v[index] = angular_velocity_y;
  calibrate_w[index] = angular_velocity_z;

}

void Controller::bias_calibrate()
{
  if (a_x_misalignment.size() == 0 || a_y_misalignment.size() == 0 || a_z_misalignment.size() == 0) {
      std::cerr << "Bias could not be calculated: Misalignment arrays are empty!" << std::endl;
      return;
  }

  // Accumulate sums of the arrays
  double sum_x = std::accumulate(a_x_misalignment.begin(), a_x_misalignment.end(), 0.0);
  double sum_y = std::accumulate(a_y_misalignment.begin(), a_y_misalignment.end(), 0.0);
  double sum_z = std::accumulate(a_z_misalignment.begin(), a_z_misalignment.end(), 0.0);

  double sum_theta = std::accumulate(calibrate_theta.begin(), calibrate_theta.end(), 0.0);
  double sum_psi = std::accumulate(calibrate_psi.begin(), calibrate_psi.end(), 0.0);
  double sum_phi = std::accumulate(calibrate_phi.begin(), calibrate_phi.end(), 0.0);

  double sum_u = std::accumulate(calibrate_u.begin(), calibrate_u.end(), 0.0);
  double sum_v = std::accumulate(calibrate_v.begin(), calibrate_v.end(), 0.0);
  double sum_w = std::accumulate(calibrate_w.begin(), calibrate_w.end(), 0.0);

  // Calculate the biases
  offset_and_turnon_bias_x = sum_x / a_x_misalignment.size();  // Ensure you're dividing by the actual size of the array
  offset_and_turnon_bias_y = sum_y / a_y_misalignment.size();
  offset_and_turnon_bias_z = sum_z / a_z_misalignment.size();

  angular_velocity_bias_u = sum_u / calibrate_u.size();
  angular_velocity_bias_v = sum_v / calibrate_v.size();
  angular_velocity_bias_w = sum_w / calibrate_w.size();

  bias_ins_theta = sum_theta / calibrate_theta.size();
  bias_ins_phi = sum_phi / calibrate_phi.size();
  bias_ins_psi = sum_psi / calibrate_psi.size();

  publish_data.accel_bias = {offset_and_turnon_bias_x,offset_and_turnon_bias_y,offset_and_turnon_bias_z};
  publish_data.gyro_bias = {angular_velocity_bias_u,angular_velocity_bias_v,angular_velocity_bias_w};
  publish_data.ins_bias = {bias_ins_theta,bias_ins_phi,bias_ins_psi};


}

void Controller::imu_error_correction()
{

  
  linear_acceleration_S_x = linear_acceleration_S_x - offset_and_turnon_bias_x;
  linear_acceleration_S_y = linear_acceleration_S_y - offset_and_turnon_bias_y;
  linear_acceleration_S_z = linear_acceleration_S_z - offset_and_turnon_bias_z;

  angular_velocity_x = angular_velocity_x - angular_velocity_bias_u;
  angular_velocity_y = angular_velocity_y - angular_velocity_bias_v;
  angular_velocity_z = angular_velocity_z - angular_velocity_bias_w;

  theta_ins = theta_ins - bias_ins_theta;
  phi_ins = phi_ins - bias_ins_phi;
  psi_ins = psi_ins - bias_ins_psi;

  publish_data.accel_bias_rem = {linear_acceleration_S_x,linear_acceleration_S_y,linear_acceleration_S_z};
  publish_data.gyro_bias_rem = {angular_velocity_x,angular_velocity_y,angular_velocity_z};
  publish_data.ins_bias_rem = {theta_ins,phi_ins,psi_ins};



}
         
void Controller::integrate()
{

  #ifdef EULER_INTEGRATION

  integrated_theta = prev_theta + (imu_dt * angular_velocity_x);
  integrated_phi = prev_phi + (imu_dt * angular_velocity_y);
  integrated_psi = prev_psi + (imu_dt * angular_velocity_z);

  prev_theta = integrated_theta;
  prev_phi = integrated_phi;
  prev_psi = integrated_psi;

  #endif
  
}

void Controller::remove_gravity()
{

  bool using_ins = true;

  if (using_ins = false){
  // angle is in radians (i think? check!!! the numbers were just smol)
    theta_rg = integrated_theta;
    phi_rg   = integrated_phi;
    psi_rg   = integrated_psi;
 
  }
  else {

    theta_rg = theta_ins;
    phi_rg   = phi_ins;
    psi_rg   = psi_ins;

  }

  a_x_g_corrected = linear_acceleration_S_x - cos(theta_rg) * sin(phi_rg) * GRAVITY;
  a_y_g_corrected = linear_acceleration_S_y + sin(theta_rg) * GRAVITY;
  a_z_g_corrected = linear_acceleration_S_z + cos(theta_rg) * cos(phi_rg) * GRAVITY;

  publish_data.accel_g_corr = {a_x_g_corrected,a_y_g_corrected,a_z_g_corrected};

}

void Controller::control_1dof()
{
  // 1DOF circular arm test

  double bar_length = 0.639; // bar length, meters
  double bar_mass = 0.39; // bar mass, kg

  // Implement proportion of gains for inertial stabilizing vs relative positional control
  
  // measured position of imu from where torque is being applied
  pm = bar_length; 

  desired_change = 0.0; // DESIRED_DISTANCE is normally 0.5 m for 3D, here we want norm around 0 change in Z

  angle  = encoder_position / 2*M_PI;

  z_dev = pm * sin(angle); // Change in z-position

  pr_err = z_dev - desired_change;

  f_i = -ka*a_z_g_corrected;

  control_torque = bar_length * f_i * sin(angle);

  SendControlMessage(control_torque);
}

void Controller::control_testing(double iter_val)
{
  double bar_length = 0.639; // bar length, meters
  double bar_mass = 0.39; // bar mass, kg
  
  double alpha = 0.2;
  double beta = 2*M_PI / (7.5 / 1);
  double test_angle = atan(beta * alpha * cos(beta*iter_val));
  double test_accel = -std::pow(beta, 2) * alpha * sin(beta*iter_val);

  f_i = -ka*test_accel;

  control_torque = bar_length * f_i * sin(test_angle);

  SendControlMessage(8*control_torque);
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
  quiet = true;
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
  quiet = true;
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

void Controller::cbINS(const  icarus_arm_control::msg::DIDINS1::SharedPtr did_ins1)
{
    // Store angular velocity values into class members
    theta_ins = did_ins1->theta[0];
    phi_ins = did_ins1->theta[1];
    psi_ins = did_ins1->theta[2];
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

void Controller::publish_imu()
{
  pub_imu_ins_->publish(publish_data);
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