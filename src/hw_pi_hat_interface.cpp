#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "ros2_hw_pi_hat_interface/ros2_hw_pi_hat_interface.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_board>());
  rclcpp::shutdown();
  return 0;
}
