#include "rclcpp/rclcpp.hpp"
#include "imu_pimu_listener.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPimuListener>();

    rclcpp::Rate loop_rate(10); // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->integrate();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
