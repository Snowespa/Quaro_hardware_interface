#ifndef __QUARO_HARDWARE_INTERFACE_QUARO_SYSTEM_HPP__
#define __QUARO_HARDWARE_INTERFACE_QUARO_SYSTEM_HPP__

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <hw_pi_hat_interface/board.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace quaro_hardware_interface
{

class QuaroSystem: public hardware_interface::SystemInterface {
public:
  QuaroSystem() = default;
  ~QuaroSystem() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo &info
  ) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &previous_state
  ) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &previous_state
  ) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &previous_state
  ) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &previous_state
  ) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &previous_state
  ) override;

  // hardware_interface::CallbackReturn on_error(
  //   const rclcpp_lifecycle::State &previous_state
  // ) override;

  // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &time,
    const rclcpp::Duration &period
  ) override;

  hardware_interface::return_type write(
    const rclcpp::Time &time,
    const rclcpp::Duration &period
  ) override;

private:

// hardware
std::unique_ptr<Board> board_;

// joints 0-11

float joint_1_pos_cmd_{0.0};
float joint_1_pos_state_{0.0};

// imu - MPU6050 returns 6 floats a_x, a_y, a_z, v_roll, v_pitch, v_yaw
float imu_lin_acc_x_{0.0};
float imu_lin_acc_y_{0.0};
float imu_lin_acc_z_{0.0};

float imu_ang_vel_x_{0.0};
float imu_ang_vel_y_{0.0};
float imu_ang_vel_z_{0.0};

// battery
float battery_voltage_{0.0};

// Leds
struct RGBLed {
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};
};
// led
std::array<RGBLed, 3> rgb_leds_;

// buzzer
float buz_on_time_{0.0};
float buz_off_time_{0.0};
float buz_freq_{0.0};
float buz_repeat_{0.0};

};

}

#endif // __QUARO_HARDWARE_INTERFACE_QUARO_SYSTEM_HPP__