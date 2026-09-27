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
    const hardware_interface::HardwareComponentInterfaceParams & params
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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &time,
    const rclcpp::Duration &period
  ) override;

  hardware_interface::return_type write(
    const rclcpp::Time &time,
    const rclcpp::Duration &period
  ) override;

private:

static constexpr double PI = 3.14159265358979323846;
static constexpr double SERVO_CENTER = 500.0;
static constexpr double SERVO_UNIT_PER_RAD = 1000.0 / (4.0 / 3.0 * PI);
// hardware
std::unique_ptr<Board> board_;

static constexpr std::size_t NUM_JOINTS = 12;
std::array<uint8_t, NUM_JOINTS> servo_ids_ {
  0, 1, 2,
  3, 4, 5,
  6, 7, 8,
  9, 10, 11,
};
// joints 0-11
std::array<double, NUM_JOINTS> joints_pos_cmds_{};
std::array<double, NUM_JOINTS> joints_pos_states_{};
uint16_t servo_max = 1000;
uint16_t servo_min = 0;

// imu - MPU6050 returns 6 floats a_x, a_y, a_z, v_roll, v_pitch, v_yaw
std::array<double, 3> imu_acc_{};
std::array<double, 3> imu_ang_vel_{};

// battery
double battery_voltage_{0.0};

// --------------------------------------------------------------------------
// Servo feedback thread
// --------------------------------------------------------------------------

std::thread servo_feedback_thread_;
std::atomic<bool> servo_feedback_running_{false};

// Written by servo feedback thread.
// Only copied into joint_positions_ by read().
std::array<double, NUM_JOINTS> servo_feedback_positions_{};
std::array<bool, NUM_JOINTS> servo_feedback_valid_{};

mutable std::mutex servo_feedback_mutex_;

void servo_feedback_loop();

// --------------------------------------------------------------------------
// Startup state
// --------------------------------------------------------------------------

// Prevents the first write() from commanding zero before we have read all
// servo positions. Commands are initialised to the actual servo positions.
bool commands_initialized_{false};


// --------------------------------------------------------------------------
// Conversion helpers
// --------------------------------------------------------------------------
std::optional<uint16_t> joint_to_servo_position(double joint_position);

std::optional<double> servo_to_joint_position(int16_t servo_position);

};

}

#endif // __QUARO_HARDWARE_INTERFACE_QUARO_SYSTEM_HPP__