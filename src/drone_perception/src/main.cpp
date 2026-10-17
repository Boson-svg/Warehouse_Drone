#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drone_perception/qr_vision_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    rclcpp::spin(std::make_shared<QrVisionNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(
        rclcpp::get_logger("qr_vision_node"),
        "Fatal error during startup: %s",
        e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
