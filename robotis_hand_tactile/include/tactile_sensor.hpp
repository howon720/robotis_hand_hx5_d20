#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hx5d20_struct.h"
#include "param.h"
#include "robotis_interfaces/msg/hand_pressures.hpp"

#include <optional>

namespace robotis_hand_tactile {

/**
 * @brief Handles tactile sensor parsing, filtering, baseline compensation, and CoP correction decision.
 */
class TactileSensor {
public:
  using HandPressuresMsg = robotis_interfaces::msg::HandPressures;
  using HandPressuresPtr = HandPressuresMsg::SharedPtr;

  TactileSensor(const rclcpp::Logger& logger, const rclcpp::Clock::SharedPtr& clock);

  /**
   * @brief Set tactile sensor parameters.
   */
  void set_params(const robotis_hand_tactile::Params& params);

  /**
   * @brief Check tactile pressure message size.
   */
  bool check_msg(const HandPressuresPtr msg) const;

  /**
   * @brief Convert tactile pressure message into internal sensor array.
   */
  SensorArray parse_sensors(const HandPressuresPtr msg) const;

  /**
   * @brief Update tactile pressure, baseline, filtered force, and CoP state.
   */
  void update_pressure(FingerArray& fingers, bool& baseline, const SensorArray& sensors);

  /**
   * @brief Select CoP correction direction from calculated tactile information.
   */
  std::optional<CorrectionDecision> pick_correction(const CopInfo& info) const;

private:
  /**
   * @brief Initialize 3x3 tactile cell coordinates.
   */
  void init_tactiles();

  /**
   * @brief Collect and update tactile baseline values.
   */
  bool update_baseline(FingerArray& fingers, bool& baseline, const SensorArray& sensors);

  /**
   * @brief Apply baseline compensation and EMA filter to tactile pressure.
   */
  PressureArray filter_pressure(FingerData& finger, const Hx5d20SensorData& sensor) const;

  /**
   * @brief Calculate total tactile force from filtered pressure.
   */
  double calc_total_force(const PressureArray& pressure) const;

  /**
   * @brief Calculate center of pressure and correction cost.
   */
  CopInfo calc_cop(int finger_idx, const PressureArray& pressure) const;

  /**
   * @brief Update one finger tactile state.
   */
  void update_finger_state(int finger_idx, FingerData& finger, const Hx5d20SensorData& sensor) const;

  /**
   * @brief Clamp value between minimum and maximum.
   */
  double clamp(double v, double min_v, double max_v) const;

private:
  // ROS interfaces
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Parameters
  robotis_hand_tactile::Params param;

  // Tactile cell coordinates
  std::array<std::pair<double, double>, tactiles_num> tactile_xy_{};

  // Tactile sensor size
  double tactile_x_{0.02}; // 2 cm
  double tactile_y_{0.02}; // 2 cm

  // Filtering and baseline
  double ema_alpha_{0.2};
  int baseline_sample_count_{30};
};

} // namespace robotis_hand_tactile