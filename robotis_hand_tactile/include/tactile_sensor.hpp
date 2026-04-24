#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hx5d20_struct.h"
#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "param.h"

#include <optional>

namespace robotis_hand_tactile {

class TactileSensor {
public:
  using HandPressuresMsg = robotis_interfaces::msg::HandPressures;
  using HandPressuresPtr = HandPressuresMsg::SharedPtr;

  TactileSensor(const rclcpp::Logger& logger, const rclcpp::Clock::SharedPtr& clock);

  bool check_msg(const HandPressuresPtr msg) const;
  SensorArray parse_sensors(const HandPressuresPtr msg) const;

  void update_pressure(FingerArray& fingers, bool& baseline, const SensorArray& sensors);
  std::optional<CorrectionDecision> pick_correction(const CopInfo& info) const;
  void set_params(const robotis_hand_tactile::Params& params);

private:
  void init_tactiles();
  bool update_baseline(FingerArray& fingers, bool& baseline, const SensorArray& sensors);
  PressureArray filter_pressure(FingerData& finger, const Hx5d20SensorData& sensor) const;
  double calc_total_force(const PressureArray& pressure) const;
  CopInfo calc_cop(int finger_idx, const PressureArray& pressure) const;
  void update_finger_state(int finger_idx, FingerData& finger, const Hx5d20SensorData& sensor) const;
  double clamp(double v, double min_v, double max_v) const;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  robotis_hand_tactile::Params param;

  std::array<std::pair<double, double>, tactiles_num> tactile_xy_{};

  double tactile_x_{0.02}; // 2cm
  double tactile_y_{0.02}; // 2cm

  double min_force_for_correction_{10.0};
  double cost_threshold_{0.1};

  double ema_alpha_{0.2}; // 이전 센서값 비례
  int baseline_sample_count_{30};
};

} // namespace robotis_hand_tactile