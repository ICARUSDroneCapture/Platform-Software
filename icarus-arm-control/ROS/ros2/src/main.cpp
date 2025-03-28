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

    while (ok())
    {
        printf("Entered while loop.\n");
        spin_some(thing->nh_);
        printf("Spin call worked.\n");
        thing->update();
        printf("Controller update worked.\n");
    }

    rclcpp::shutdown();
    
    return 0;
 }
 