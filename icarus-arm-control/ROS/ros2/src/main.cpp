#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include "socket_can.hpp"
#include <thread>

int main(int argc, char* argv[]) {

    // (void) argc;
    // (void) argv;

    // printf("Here will be the controller listener.\nThis node will do integration, gravity correction and control law calculations.\n");
    // return 0;

    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<Controller>());
    // rclcpp::shutdown();
    // return 0;

    rclcpp::init(argc, argv);
    EpollEventLoop event_loop;
    auto can_node = std::make_shared<ODriveCanNode>("ODriveCanNode");

    if (!can_node->init(&event_loop)) return -1;

    std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });
    rclcpp::spin(can_node);
    can_node->deinit();
    rclcpp::shutdown();
    return 0;
}
