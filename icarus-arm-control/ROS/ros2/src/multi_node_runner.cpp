#include "rclcpp/rclcpp.hpp"

// Include headers for your nodes
#include "controller.hpp"
#include "imu_logger.hpp"
#include "torque_logger.hpp"
#include "imu_pimu_listener.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create each node
    auto controller_node = std::make_shared<Controller>();
    auto imu_logger_node = std::make_shared<ImuLogger>();
    auto integrator_node = std::make_shared<IntegratorNode>();
    auto torque_logger_node = std::make_shared<TorqueLogger>();

    // Spin them in a loop using spin_some()
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller_node);
    executor.add_node(imu_logger_node);
    executor.add_node(integrator_node);
    executor.add_node(torque_logger_node);

    rclcpp::Rate rate(100);  // 100 Hz loop (adjust as needed)
    while (rclcpp::ok()) {
        executor.spin_some();  // Handle any available callbacks
        rate.sleep();          // Sleep to control CPU usage
    }

    rclcpp::shutdown();
    return 0;
}
