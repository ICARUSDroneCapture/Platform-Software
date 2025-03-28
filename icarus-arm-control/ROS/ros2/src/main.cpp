 /**
 * @file main.cpp
 *
 * @brief Controller listener node main function, will drive the primary functionality 
 * of the entire controlling system.
 *
 */

 #include "controller.hpp"

 int main(int argc, char**argv)
 {
    // Universal rclcpp init
    rclcpp::init(argc, argv);
    
    auto controller_node = std::make_shared<Controller>();

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
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);
    isROS.initialize();

    bool success = false;
    unsigned int startTimeMs = current_timeMs(), prevTimeMs = 0, nowTimeMs;
	while((nowTimeMs = current_timeMs()) - startTimeMs < 5000)
	{
            isROS.update();
	    rclcpp::spin_some(controller_node);
        if (controller_node->did_rx_pimu_) {
            success = true;
            break;
        } else {
            // check regularly, but don't print regularly..
            SLEEP_MS(200);
            if (prevTimeMs / 1000 != nowTimeMs / 1000) {
                TEST_COUT << "waiting...  (time: " << nowTimeMs << ")" << std::endl;
                prevTimeMs = nowTimeMs;
            }
        }
    }

    ASSERT_TRUE( success );
    isROS.terminate();

    rclcpp::shutdown();
    
    return 0;
 }
 