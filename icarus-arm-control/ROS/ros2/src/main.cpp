/**
 * @file main.cpp
 *
 * @brief Controller listener node main function, will drive the primary functionality 
 * of the entire controlling system.
 *
 */

 #include "controller.hpp"

 using namespace rclcpp;

 int main(int argc, char**argv)
 {
    Controller* thing;

    // Universal rclcpp init
    init(argc, argv);

    printf("Here will be the controller listener.\nThis node will do gravity correction and control law calculations.\n");

    // Initialize our controller object
    thing->initialize();

    printf("Initialization finished??.\n");

    spin(thing)

    // while (ok())
    // {
    //     printf("Entered while loop.\n");
    //     spin_some(thing->cl_);
    //     printf("Spin call worked.\n");
    //     thing->update();
    //     printf("Controller update worked.\n");
    // }

    printf("Spin done, shutdown to be called on deconstructor.\n");
    
    return 0;

    // auto controller_node = std::make_shared<Controller>();

    // if (!controller_node->initialize()) return -1;

    // std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });
    // rclcpp::spin(can_node);
    // can_node->deinit();
    // rclcpp::shutdown();
    // return 0;
 }
 