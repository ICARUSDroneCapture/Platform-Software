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

    int refresh_wait = 50;
    

    while (ok())
    {
        rclcpp::spin_some(controller_node);

        controller_node->controller_dt = refresh_wait;

        controller_node->step();

        // periodic print, add update function
        SLEEP_MS(refresh_wait);
        if (prevTimeMs / 1000 != nowTimeMs / 1000) {
            if (!controller_node->plot_quiet) {
                controller_node->plot(controller_node->nodeStartTime);
            }
            RCLCPP_INFO(rclcpp::get_logger("controller"),"running...  (time: %u)\n\n", nowTimeMs);
            prevTimeMs = nowTimeMs;
        }
        nowTimeMs = current_timeMs();
    }

    rclcpp::shutdown();
    
    return 0;
 }
 