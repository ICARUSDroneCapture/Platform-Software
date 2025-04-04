#include "rclcpp/rclcpp.hpp"
#include "imu_pimu_subscriber.cpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImuPimuSubscriber>();

    // Run the node in a loop and process data
    rclcpp::Rate rate(10); // 10 Hz loop
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Access values and do computations here
        RCLCPP_INFO(node->get_logger(), "IMU Orientation: [%.2f, %.2f, %.2f]",
                    node->orientation_x, node->orientation_y, node->orientation_z);

        RCLCPP_INFO(node->get_logger(), "PIMU DTheta: [%.2f, %.2f, %.2f]",
                    node->dtheta_x, node->dtheta_y, node->dtheta_z);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
