#include <chrono>
#include <memory>

#include "ros2_hw_pi_hat_interface/ros2_hw_pi_hat_interface.hpp"


using namespace std::chrono_literals;

ros2_board::ros2_board() : Node("ros2_board"), board() {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    board.setRecieve(true);
    auto imu_cb = [this] () {
        this->publish_imu_msg();
    };
    timer_ = this->create_wall_timer(100ms, imu_cb);
}

ros2_board::~ros2_board() {
    board.setRecieve(false);
}

void ros2_board::publish_imu_msg() {
    imu_data_ = board.getIMU();
    if (!imu_data_.has_value()) {
        return;
    }

    auto *imu = imu_data_.value();
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    msg.angular_velocity.x = imu[0];
    msg.angular_velocity.y = imu[1];
    msg.angular_velocity.z = imu[2];

    msg.linear_acceleration.x = imu[3];
    msg.linear_acceleration.y = imu[4];
    msg.linear_acceleration.z = imu[5];

    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = 0.0;
    msg.linear_acceleration_covariance[0] = 0.0;

    publisher_->publish(msg);
}