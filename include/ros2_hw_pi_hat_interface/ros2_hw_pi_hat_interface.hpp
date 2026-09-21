#ifndef __ROS2_HW_PI_HAT_INTERFACE__
#define __ROS2_HW_PI_HAT_INTERFACE__

#include <optional>

#include <hw_pi_hat_interface/board.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ros2_board : public rclcpp::Node {
public:
  ros2_board();
  ~ros2_board() override;

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  Board board;
  std::optional<float *> imu_data_;
  void publish_imu_msg();
};

#endif // __ROS2_HW_PI_HAT_INTERFACE__