 /**
 * @file main.cpp
 *
 * @brief Controller listener node main function, will drive the primary functionality 
 * of the entire controlling system.
 *
 */

#include "controller.hpp"

#define NDEBUG

#ifndef NDEBUG
#define ASSERT_EX(condition, statement) \
    do { \
        if (!(condition)) { statement; assert(condition); } \
    } while (false)
#else
#define ASSERT_EX(condition, statement) ((void)0)
#endif

 int main(int argc, char**argv)
 {
    // Universal rclcpp init
    rclcpp::init(argc, argv);
    
    auto controller_node = std::make_shared<Controller>();
    // auto can_node = std::make_shared<ODriveCanNode>("ODriveCanNode");

    // Initialize our controller object
    controller_node->init();

    bool success = false;
    unsigned int startTimeMs = current_timeMs(), prevTimeMs = 0, nowTimeMs;
    unsigned int control_step_finish = current_timeUs(), control_step_start = 0, control_step_time;
    unsigned int spin_finish = current_timeUs(), spin_start = 0, spin_time;


    ASSERT_EX(success, std::cout << "IMU RX fail.\n");

    controller_node->nodeStartTime = static_cast<double>( current_timeMs() );
    
    int data_points = 5000;
    int i = 0;
    while (i <= data_points) {

        std::cout << "Calibrating... " << std::to_string(i) << "/" << std::to_string(data_points) << std::endl;

        rclcpp::spin_some(controller_node);
        controller_node->imu_configure(i);
        i += 1;
    }

    rclcpp::spin_some(controller_node);
    controller_node->bias_calibrate();

    int refresh_wait = 3;

    int step_count = 0;
    unsigned int start_time = current_timeUs();
    unsigned int total_time = 0;
    unsigned int step_time_running_average = 0;

    while (ok())
    {
        spin_start = current_timeUs();
        
        rclcpp::spin_some(controller_node);
        
        spin_finish = current_timeUs();

        spin_time = spin_finish - spin_start;
        
        
        controller_node->controller_dt = refresh_wait;

        control_step_start = current_timeUs();

        controller_node->step();

        control_step_finish = current_timeUs();

        control_step_time = control_step_finish - control_step_start;

        // periodic print, add update function
        SLEEP_MS(refresh_wait);
        if (prevTimeMs / 1000 != nowTimeMs / 1000) {
            if (!controller_node->plot_quiet) {
                controller_node->plot(controller_node->nodeStartTime);
            }
            RCLCPP_INFO(rclcpp::get_logger("controller"),"running...  (time: %u)\n\n", nowTimeMs);
            RCLCPP_INFO(rclcpp::get_logger("controller"),"spin time...  (time: %u Us)\n\n", spin_time);
            RCLCPP_INFO(rclcpp::get_logger("controller"),"control step time...  (time: %u Us)\n\n", control_step_time);
            RCLCPP_INFO(rclcpp::get_logger("controller"),"total step time...  (time: %u Us)\n\n", step_time_running_average);
            prevTimeMs = nowTimeMs;
        }
        nowTimeMs = current_timeMs();

        step_count = step_count + 1;
        total_time = current_timeUs() - start_time;

        // Microsecond running average of step time
        step_time_running_average = total_time / step_count;

    }

    rclcpp::shutdown();
    
    return 0;
 }
 