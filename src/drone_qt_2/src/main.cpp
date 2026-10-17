#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "drone_qt_2/airborne_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto airborne_node = std::make_shared<AirborneNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(airborne_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}