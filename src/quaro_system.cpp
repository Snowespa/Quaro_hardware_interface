#include "quaro_hardware_interface/quaro_system.hpp"

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <math.h>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <hw_pi_hat_interface/board.hpp>

namespace quaro_hardware_interface
{

hardware_interface::CallbackReturn QuaroSystem::on_init(
  const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Configured Hiwonder communication board."
  );
  return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn QuaroSystem::on_configure(
  const rclcpp_lifecycle::State &previous_state) {
  board_ = std::make_unique<Board>();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuaroSystem::on_activate(
  const rclcpp_lifecycle::State &previous_state) {
  if (!board_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("QuaroSystem"), "Cannot activate: Board is not configured"
    );
    return hardware_interface::CallbackReturn::ERROR;
  }
  board_->setRecieve(true);
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Activating Quaro System."
  );
  return hardware_interface::CallbackReturn::SUCCESS;
} 

hardware_interface::CallbackReturn QuaroSystem::on_deactivate(
  const rclcpp_lifecycle::State &previous_state) {
  
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Deactivation Quaro System."
  );
  board_->setRecieve(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuaroSystem::on_shutdown(
  const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Quaro system shuting down."
  );
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuaroSystem::on_cleanup(
  const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Cleaning Quaro System up."
  );
  board_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// std::vector<hardware_interface::StateInterface> QuaroSystem::export_state_interfaces() {
//   std::vector<hardware_interface::StateInterface> state_interface;

//   state_interface.emplace_back(

//   )
// }

// std::vector<hardware_interface::CommandInterface> QuaroSystem::export_command_interface() {

// }

hardware_interface::return_type QuaroSystem::read(
  const rclcpp::Time &time, const rclcpp::Duration &period) {
  
  std::optional<int16_t> pos = board_->getServoPos(0);
  if (pos)
    joint_1_pos_state_ = static_cast<float>(pos.value());

  std::optional<float *> imu_data = board_->getIMU();
  if (imu_data) {
    float *data = imu_data.value();
    imu_lin_acc_x_ = data[0];
    imu_lin_acc_y_ = data[1];
    imu_lin_acc_z_ = data[2];

    imu_ang_vel_x_ = data[3];
    imu_ang_vel_y_ = data[4];
    imu_ang_vel_z_ = data[5];
  }

  std::optional<uint16_t> battery = board_->getBattery();
  if (battery) {
    battery_voltage_ = static_cast<float>(battery.value())/1000.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuaroSystem::write(
  const rclcpp::Time &time, const rclcpp::Duration &period) {
  return hardware_interface::return_type::OK;
}


}

PLUGINLIB_EXPORT_CLASS(
  quaro_hardware_interface::QuaroSystem,
  hardware_interface::SystemInterface
)


