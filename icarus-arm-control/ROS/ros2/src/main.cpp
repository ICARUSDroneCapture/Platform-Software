 /**
 * @file main.cpp
 *
 * @brief Controller listener node main function, will drive the primary functionality 
 * of the entire controlling system.
 *
 */

#include "controller.hpp"
#include "inertial_sense_ros.h"

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

    //rclcpp::spin_some(controller_node);
    std::string yaml = "topic: \"inertialsense\"\n"
                       "port: [/dev/ttyACM0, /dev/ttyACM1, /dev/ttyACM2]\n"
                       "baudrate: 921600\n"
                       "\n"
                       "ins:\n"
                       "  navigation_dt_ms: 16                          # EKF update period.  uINS-3: 4  default, 1 max.  Use `msg/ins.../period` to reduce INS output data rate."
                       "\n"
                       "sensors:\n"
                       "  messages:  \n"
                       "    pimu:             # Publish preintegrated IMU delta theta and delta velocity\n"
                       "      topic: \"pimu\"\n"
                       "      enable: true\n"
                       "      period: 1\n";

    YAML::Node config = YAML::Load(yaml);
    ASSERT_EX(config.IsDefined(), std::cout << "Unable to parse YAML file. Is the file valid?\n");

    InertialSenseROS isROS(config);
    isROS.initialize();

    bool success = false;
    unsigned int startTimeMs = current_timeMs(), prevTimeMs = 0, nowTimeMs;
	while((nowTimeMs = current_timeMs()) - startTimeMs < 5000)
	{
        isROS.update();
	    rclcpp::spin_some(controller_node);

        controller_node->step();

        if (controller_node->did_rx_pimu_) {
            success = true;
            break;
        } else {
            // check regularly, but don't print regularly..
            SLEEP_MS(200);
            if (prevTimeMs / 1000 != nowTimeMs / 1000) {
                RCLCPP_INFO(rclcpp::get_logger("init"),"waiting...  (time: %u)\n", nowTimeMs);
                prevTimeMs = nowTimeMs;
            }
        }
    }

    ASSERT_EX(success, std::cout << "IMU RX fail.\n");

    double startTime = static_cast<double>( current_timeMs() );

    if (ok()) {
        isROS.update();
        rclcpp::spin_some(controller_node);
        controller_node->imu_configure();
    } else {
        RCLCPP_INFO(rclcpp::get_logger("error"),"\n\n");
        success = false;
        ASSERT_EX(success, std::cout << "imu ready, ROS failure... exiting.\n");
    }

    while (ok())
    {
        isROS.update();
        rclcpp::spin_some(controller_node);

        controller_node->step();

        // periodic print, add update function
        SLEEP_MS(500);
        if (prevTimeMs / 1000 != nowTimeMs / 1000) {
            if (!controller_node->plot_quiet) {
                controller_node->plot(startTime);
            }
            RCLCPP_INFO(rclcpp::get_logger("controller"),"running...  (time: %u)\n\n", nowTimeMs);
            prevTimeMs = nowTimeMs;
        }
        nowTimeMs = current_timeMs();
    }

    rclcpp::shutdown();
    
    return 0;
 }
 