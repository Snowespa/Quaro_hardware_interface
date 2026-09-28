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
  const hardware_interface::HardwareComponentInterfaceParams & params) {
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  // Check if all joints are defined.
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("QuaroSyste"), "Expected %zu joints, got %zu", NUM_JOINTS, info_.joints.size()
    );
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Check the number of sensors (IMU, Battery)
  if (info_.sensors.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("QuaroSystem"), "Expected 2 sensors (imu, battery), got %zu", info_.sensors.size()
    );
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Configured Hiwonder communication board."
  );
  return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn QuaroSystem::on_configure(
  const rclcpp_lifecycle::State &) {
  board_ = std::make_unique<Board>();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuaroSystem::on_activate(
  const rclcpp_lifecycle::State &) {
  if (!board_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("QuaroSystem"), "Cannot activate: Board is not configured"
    );
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Activating Quaro System."
  );

  board_->setRecieve(true);
  // Reset startup state.
  commands_initialized_ = false;

  {
    std::lock_guard<std::mutex> lock(servo_feedback_mutex_);
    servo_feedback_valid_.fill(false);
  }

  servo_feedback_running_.store(true);
  servo_feedback_thread_ = std::thread(&QuaroSystem::servo_feedback_loop, this);

  return hardware_interface::CallbackReturn::SUCCESS;
} 

hardware_interface::CallbackReturn QuaroSystem::on_deactivate(
  const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Deactivation Quaro System."
  );
  servo_feedback_running_.store(false);

  if (servo_feedback_thread_.joinable()) {
    servo_feedback_thread_.join();
  }

  if (board_) {
    for (const auto id : servo_ids_) {
      board_->setServoTorque(id, false);
    }
    // Stop board receive threads.
    board_->setRecieve(false);
  }

  board_->setRecieve(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuaroSystem::on_shutdown(
  const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Shuting down Quaro Hardware."
  );
  servo_feedback_running_.store(false);

  if (servo_feedback_thread_.joinable()) {
    servo_feedback_thread_.join();
  }

  if (board_) {
    for (const auto id : servo_ids_) {
      board_->setServoTorque(id, false);
    }
    // Stop board receive threads.
    board_->setRecieve(false);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuaroSystem::on_cleanup(
  const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(
    rclcpp::get_logger("QuaroSystem"), "Cleaning Quaro System up."
  );
  servo_feedback_running_.store(false);

  if (servo_feedback_thread_.joinable()) {
    servo_feedback_thread_.join();
  }

  board_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QuaroSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.reserve(NUM_JOINTS + 6 + 1); // 6 for the IMU, 1 for the battery

  // Joint state interfaces.
  for (std::size_t i = 0; i < NUM_JOINTS; ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &joints_pos_states_[i]);
  }

  // IMU state interfaces.
  state_interfaces.emplace_back(
    "imu",
    "linear_acceleration.x",
    &imu_acc_[0]);

  state_interfaces.emplace_back(
    "imu",
    "linear_acceleration.y",
    &imu_acc_[1]);

  state_interfaces.emplace_back(
    "imu",
    "linear_acceleration.z",
    &imu_acc_[2]);

  state_interfaces.emplace_back(
    "imu",
    "angular_velocity.x",
    &imu_ang_vel_[0]);

  state_interfaces.emplace_back(
    "imu",
    "angular_velocity.y",
    &imu_ang_vel_[1]);

  state_interfaces.emplace_back(
    "imu",
    "angular_velocity.z",
    &imu_ang_vel_[2]);

  // Battery.
  state_interfaces.emplace_back(
    "battery",
    "battery_voltage",
    &battery_voltage_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> QuaroSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.reserve(NUM_JOINTS);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &joints_pos_cmds_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type QuaroSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!board_)
    return hardware_interface::return_type::ERROR;
  std::optional<std::array<float, 6>> imu_data = board_->getIMU();
  if (imu_data) {
    std::array<float, 6> data = imu_data.value();
    imu_acc_[0] = static_cast<double>(data[0]);
    imu_acc_[1] = static_cast<double>(data[1]);
    imu_acc_[2] = static_cast<double>(data[2]);

    imu_ang_vel_[0] = static_cast<double>(data[3]);
    imu_ang_vel_[1] = static_cast<double>(data[4]);
    imu_ang_vel_[2] = static_cast<double>(data[5]);
  }

  std::optional<uint16_t> battery_data = board_->getBattery();
  if (battery_data)
    battery_voltage_ = static_cast<double>(battery_data.value()) / 1000.0;

  {
    std::lock_guard<std::mutex> lock(servo_feedback_mutex_);

    bool all_servos_valid = true;
    // Try to read the servo values
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
      if (servo_feedback_valid_[i])
        joints_pos_states_[i] = servo_feedback_positions_[i];
      else
        all_servos_valid = false;
    }

    // Once every joint has been observed, initialise the command interfaces
    // to the real robot position. This prevents the first write() from
    // commanding every joint to zero radians.
    if (all_servos_valid && !commands_initialized_)
    {
      joints_pos_cmds_ = joints_pos_states_;
      commands_initialized_ = true;

      RCLCPP_INFO(
        rclcpp::get_logger("QuaroSystem"),
        "Initial servo positions acquired; command interfaces initialised.");
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuaroSystem::write(const rclcpp::Time &, const rclcpp::Duration & period) {
  if (!board_)
    return hardware_interface::return_type::ERROR;

  // Don't send anything until we know the actual initial joint positions.
  // This prevents an activation-time jump to zero.
  if (!commands_initialized_)
    return hardware_interface::return_type::OK;

  std::array<uint16_t, NUM_JOINTS> servo_positions{};

  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    std::optional<uint16_t> servo_position = joint_to_servo_position(joints_pos_cmds_[i]);
    
    if (!servo_position) {
      RCLCPP_ERROR(
        rclcpp::get_logger("QuaroSystem"),
        "Joint %zu command %.4f rad is outside the supported "
        "LX-224HV servo range.",
        i,
        joints_pos_cmds_[i]);

      return hardware_interface::return_type::ERROR;
    }

    servo_positions[i] = servo_position.value();
  }

  std::vector<uint8_t> ids(servo_ids_.begin(), servo_ids_.end());
  std::vector<uint16_t> positions(servo_positions.begin(), servo_positions.end());

  // Use one control period as the servo move duration.
  // At 100 Hz this gives a 10 ms move time.
  const double duration = std::clamp(period.seconds(), 0.0, 30.0);

  board_->setServoPos(ids, positions, static_cast<float>(duration));

  return hardware_interface::return_type::OK;
}

void QuaroSystem::servo_feedback_loop() {
  constexpr auto poll_interval = std::chrono::milliseconds(50);
  
  while (servo_feedback_running_.load()){
    auto next_cycle = std::chrono::steady_clock::now() + poll_interval;

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
      if (std::optional<double> position = board_->getServoPos(servo_ids_[i])) {
        if(std::optional<double> joint_position = servo_to_joint_position(position.value())) {
          std::lock_guard<std::mutex> lock(servo_feedback_mutex_);
          servo_feedback_positions_[i] = joint_position.value();
          servo_feedback_valid_[i] = true;
        }
      }
    }
    std::this_thread::sleep_until(next_cycle);
  }
}

std::optional<uint16_t> QuaroSystem::joint_to_servo_position(double joint_pos) {
  const double ticks = joint_pos * SERVO_UNIT_PER_RAD + SERVO_CENTER;
  if (ticks < 0.0 || ticks > 1000.0)
    return std::nullopt;
  return static_cast<uint16_t>(std::lround(ticks));
}

std::optional<double> QuaroSystem::servo_to_joint_position(int16_t servo_pos) {
  if (servo_pos < 0 || servo_pos > 1000)
    return std::nullopt;
  return (static_cast<double>(servo_pos) - SERVO_CENTER) / SERVO_UNIT_PER_RAD;
}

}

PLUGINLIB_EXPORT_CLASS(
  quaro_hardware_interface::QuaroSystem,
  hardware_interface::SystemInterface
)


