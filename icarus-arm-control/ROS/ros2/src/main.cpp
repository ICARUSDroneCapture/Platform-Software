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

    Initialize our controller object
    thing->initialize();

    // while (ok())
    // {
    //     spin_some(thing->nh_);
    //     thing->update();
    // }

    rclcpp::shutdown();
    
    return 0;
 }
 