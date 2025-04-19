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
    sub_pimu_               = this->create_subscription<inertial_sense_ros2::msg::PIMU>("pimu", 1, std::bind(&Controller::cbPIMU, this, std::placeholders::_1));
    sub_imu_                = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&Controller::cbIMU, this, std::placeholders::_1));
    sub_ins_                = this->create_subscription<inertial_sense_ros2::msg::DIDINS1>("ins_eul_uvw_ned", 1, std::bind(&Controller::cbINS, this, std::placeholders::_1));

    sub_motor_cntr_stat_    = this->create_subscription<odrive_can::msg::ControllerStatus>("controller_status", 1, std::bind(&Controller::cbCtrlStatus, this, std::placeholders::_1));
    sub_odrv_stat_     = this->create_subscription<odrive_can::msg::ODriveStatus>("odrive_status", 1, std::bind(&Controller::cbODrvStatus, this, std::placeholders::_1));

    sub_int_ = this->create_subscription<icarus_arm_control::msg::IntegratedAngles>("integrated_angles", 1, std::bind(&Controller::cbInt, this, std::placeholders::_1));

    sub_gain_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("gain_topic", 10,std::bind(&Controller::cbGain, this, std::placeholders::_1));

    msg_ctrl.control_mode = 1;
    msg_ctrl.input_mode = 1;
    pub_motor_cntr_msg_     = this->create_publisher<odrive_can::msg::ControlMessage>("control_message", 1);
    pub_imu_ins_            = this->create_publisher<icarus_arm_control::msg::PublishData>("publish_data", 1);

    
}

void Controller::step()
{



  // Correct for imu sensor errors
  // imu_error_correction();

  // Integrate IMU angular velocity
  //integrate();

  // Remove gravity from acceleration values
  rotate_S_I();

  integrate_encoder();
  
  // Perform 1DOF control law

  // iter_val = iter_val + 0.01;
  // control_testing(iter_val);

  control_1dof();

  //Print
 print_data();



  publish_imu();

}

void Controller::plot(double startTime)
{
  // double currTime = static_cast<double>( current_timeMs() ) - startTime;
  // plot_ts.push_back(currTime / 1000.0);

  // plot_a_x.push_back(linear_acceleration_S_x);
  // plot_a_y.push_back(linear_acceleration_S_y);
  // plot_a_z.push_back(linear_acceleration_S_z);

  // plot_ts.erase(plot_ts.begin());

  // plot_a_x.erase(plot_a_x.begin());
  // plot_a_y.erase(plot_a_y.begin());
  // plot_a_z.erase(plot_a_z.begin());

  // auto ax1 = matplot::nexttile();
  // auto ax2 = matplot::nexttile();
  // auto ax3 = matplot::nexttile();

  // auto l1 = matplot::scatter(ax1, plot_ts, plot_a_x);  
  // auto l2 = matplot::scatter(ax2, plot_ts, plot_a_y);
  // auto l3 = matplot::scatter(ax3, plot_ts, plot_a_z);

  // xlim(ax1, {0, 180});
  // xlim(ax2, {0, 180});
  // xlim(ax3, {0, 180});

  // ylim(ax1, {-2, 2});
  // ylim(ax2, {-2, 2});
  // ylim(ax3, {-12, -8});

  // l1->marker_face(true);
  // l2->marker_face(true);
  // l3->marker_face(true);

  // matplot::hold({ax1, ax2, ax3}, matplot::on);

  // matplot::show();
  
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

  quiet = false;
  if (!quiet) {
    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\t----------------------------------\n");
    
    // RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tDesired Angle: %f (degrees) \n", (desired_angle / M_PI * 180));

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tRaw Encoder Position: %f (revs) \n", encoder_position);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tRaw Encoder Velocity: %f (revs) \n", encoder_velocity);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tTheta Angle: %f (rad) \n", theta_rg);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tPhi Angle: %f (rad) \n", phi_rg);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tPsi Angle: %f (rad) \n", psi_rg);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tScaled Angle: %f (degrees) \n", (scaled_position * 360));

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tPosition Error Correction: %f (degrees) \n", (pr_err / M_PI * 180));

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tEncoder Angle: %f (degrees) \n", (encoder_position / 50 * 360));

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tController Timestep (ms): %f\n", controller_dt);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tEncoder Error (rad): %f\n", encoder_err);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tIntegrated Encoder (rad): %f\n", integrated_enc);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tInertially Stabalizing Torque: %f\n", control_torque_fi);

    RCLCPP_INFO(rclcpp::get_logger("data"),"\t\tInputed Torque: %f\n", control_torque);
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

void Controller::integrate_encoder()
{

  encoder_err = (scaled_position * 2 * M_PI) - desired_angle;

  integrated_enc = prev_enc_ + (controller_dt/1000 * encoder_err);
  prev_enc_ = integrated_enc;

}

void Controller::rotate_S_I()
{



    theta_rg = theta_ins;
    phi_rg   = phi_ins;
    psi_rg   = psi_ins;


  // Standard 3-2-1 Euler Rotations

  // Precompute sines and cosines
  double cphi   = std::cos(phi_rg);
  double sphi   = std::sin(phi_rg);
  double ctheta = std::cos(theta_rg);
  double stheta = std::sin(theta_rg);
  double cpsi   = std::cos(psi_rg);
  double spsi   = std::sin(psi_rg);

  // ----------------------------------------------------------------
  // Rotation Matrix R = Rz(psi) * Ry(theta) * Rx(phi)
  // That is, apply heading first (about Z), pitch second (about Y),
  // roll last (about X). This transforms sensor-frame vectors
  // into the INS-output frame (Intermediate output frame).
  // ----------------------------------------------------------------

  // [Row 0]
  double r00 =  cpsi * ctheta;
  double r01 =  cpsi * stheta * sphi - spsi * cphi;
  double r02 =  cpsi * stheta * cphi + spsi * sphi;

  // [Row 1]
  double r10 =  spsi * ctheta;
  double r11 =  spsi * stheta * sphi + cpsi * cphi;
  double r12 =  spsi * stheta * cphi - cpsi * sphi;

  // [Row 2]
  double r20 = -stheta;
  double r21 =  ctheta * sphi;
  double r22 =  ctheta * cphi;

  // Multiply the rotation matrix by the sensor-frame acceleration
  double ax_i = r00 * linear_acceleration_S_x
              + r01 * linear_acceleration_S_y
              + r02 * linear_acceleration_S_z;

  double ay_i = r10 * linear_acceleration_S_x
              + r11 * linear_acceleration_S_y
              + r12 * linear_acceleration_S_z;

  double az_i = r20 * linear_acceleration_S_x
              + r21 * linear_acceleration_S_y
              + r22 * linear_acceleration_S_z;

  // Output
  a_x_g_corrected = ax_i;
  a_y_g_corrected = ay_i;
  a_z_g_corrected = az_i + GRAVITY;

  publish_data.accel_g_corr = {a_x_g_corrected, a_y_g_corrected, a_z_g_corrected};

}

void Controller::control_1dof()
{

  if (a_x_g_corrected == 0.0 && a_y_g_corrected == 0.0 && a_z_g_corrected == 0.0) {
    f_i = 0.0;
  } else if (std::isnan(a_x_g_corrected) || std::isnan(a_y_g_corrected) || std::isnan(a_z_g_corrected)) {
    f_i = 0.0;
  } else {
    f_i = ka*a_z_g_corrected;
  }


  // 1DOF circular arm test

  double bar_length = 0.639; // bar length, meters
  double bar_mass = 3.08; // bar mass, kg

  // Account for gearbox
  scaled_position = encoder_position / 50;

  // Implement proportion of gains for inertial stabilizing vs relative positional control
  
  // measured position of imu from where torque is being applied
  pm = bar_length; 

  // desired_pos = pm * sin(desired_angle); // DESIRED_DISTANCE is normally 0.5 m for 3D, here we want norm around 0 change in Z

  // z_dev = pm * sin(scaled_position); // Change in z-position

  pr_err = scaled_position * 2*M_PI - desired_angle;

  f_pr = -(kp*pr_err + ki*integrated_enc + kd*encoder_velocity);
  // f_pr = -(kp*pr_err);

  control_force = f_i;
  //  control_force = f_i;
  // control_force = f_pr;

  control_torque_fi = (bar_length * f_i) / GEAR_RATIO;

  control_torque = (bar_length * cos(theta_rg) * control_force) / GEAR_RATIO + f_pr/GEAR_RATIO;

  // Sticktion force

  // torque_stick = 0.075;
  // torque_comp = torque_stick * control_torque / abs(control_torque);

  // double holding_torque = 0.157 * sin(2 * M_PI * encoder_position / GEAR_RATIO);
  // control_torque = control_torque + holding_torque + torque_stick * control_torque / abs(control_torque);

  // control_torque = control_torque + torque_stick * control_torque / abs(control_torque);

  quiet = true;

  SendControlMessage(control_torque);
}

void Controller::control_testing(double iter_val)
{

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

void Controller::cbPIMU(const inertial_sense_ros2::msg::PIMU::SharedPtr pimu)
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

void Controller::cbINS(const  inertial_sense_ros2::msg::DIDINS1::SharedPtr did_ins1)
{
    // Store angular velocity values into class members
    theta_ins = did_ins1->theta[1];
    phi_ins = did_ins1->theta[0];
    psi_ins = did_ins1->theta[2];
}
   
void Controller::cbCtrlStatus(const  odrive_can::msg::ControllerStatus::SharedPtr ctrl_stat_)
{
    encoder_position = ctrl_stat_->pos_estimate;
    encoder_velocity = ctrl_stat_->vel_estimate;
}

void Controller::cbODrvStatus(const  odrive_can::msg::ODriveStatus::SharedPtr odrv_stat_)
{
    motor_temperature = odrv_stat_->motor_temperature;
}

void Controller::cbInt(const icarus_arm_control::msg::IntegratedAngles::SharedPtr sub_int_)
{
    // int_enc = int_->integrated_enc;
    // place holder
}

void Controller::cbGain(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 6) {
        // Update the dynamic gains
        kp = msg->data[0];
        ka = msg->data[1];
        ki = msg->data[2];
        kd = msg->data[3];
        kv = msg->data[4];
        ks = msg->data[5];

        // RCLCPP_INFO(this->get_logger(),
        //             "Dynamic Gains updated: kp = %f, ka = %f, ki = %f, kd = %f, kv = %f, ks = %f",
        //             kp, ka, ki, kd, kv, ks);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received gain message with insufficient elements.");
    }
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

double Controller::sign(double x) {
    return (x > 0) - (x < 0);
}

// Center gain function C(x) — inertial isolation
double Controller::C(double x, double w, double d) {
    double c = 0.5;
    double r_c = w/2 * c;
    int n = 4;
    double a_c = -1.0 / std::pow(std::abs(r_c - w / 2.0), n);
    double k_c = 1.0;

    double abs_xd = std::abs(x - d);

    if (abs_xd <= r_c) {
        return 1.0;
    } else if (abs_xd > r_c && abs_xd <= w / 2.0) {
        return a_c * std::pow(std::abs(x - (d + sign(x - d) * r_c)), n) + k_c;
    } else {
        return 0.0;
    }
}

// Boundary gain function B(x) — relative position control
double Controller::B(double x, double w, double d) {
    double b = 0.5;
    double r_b = w/2 * b;
    int n = 4;
    double a_b = 1.0 / std::pow(std::abs(r_b - w / 2.0), n);
    double k_b = 0.0;

    double abs_xd = std::abs(x - d);

    if (abs_xd > w / 2.0) {
        return 1.0;
    } else if (abs_xd > r_b && abs_xd <= w / 2.0) {
        return a_b * std::pow(std::abs(x - (d + sign(x - d) * r_b)), n) + k_b;
    } else {
        return 0.0;
    }
}
