// /**
//  * @file main.cpp
//  *
//  * @brief Controller listener node main function, will drive the primary functionality 
//  * of the entire controlling system.
//  *
//  */

//  #include "controller.hpp"

//  int main(int argc, char**argv)
//  {
//     // Universal rclcpp init
//     init(argc, argv);

//     printf("Here will be the controller listener.\nThis node will do gravity correction and control law calculations.\n");

//     auto controller_node = std::make_shared<Controller>();

//     // Initialize our controller object
//     if (!controller_node->initialize()) return -1;

//     printf("Controller initialization finished.\n");

//     // ROS spin our object
//     spin(controller_node);

//     printf("Spin done, shutdown to be called on deconstructor.\n");
    
//     return 0;

//  }


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

    // while (ok())
    // {
    //     spin_some(thing->nh_);
    //     thing->update();
    // }

    rclcpp::shutdown();
    
    return 0;
 }
 