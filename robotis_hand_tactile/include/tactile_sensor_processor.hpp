#pragma once

#include "rclcpp/rclcpp.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "tactile_grasp_controller.hpp"

#include <array>
#include <memory>
#include <optional>
#include <utility>

namespace robotis_hand_tactile {

typedef robotis_interfaces::msg::HandPressures::SharedPtr HandPressuresPtr;

typedef TactileGraspController Controller;
typedef TactileGraspController::Finger Finger;
typedef TactileGraspController::CopInfo CopInfo;
typedef TactileGraspController::CorrectionDecision CorrectionDecision;

typedef std::array<Finger, TactileGraspController::fingers_num> FingerArray;
typedef std::array<Hx5d20SensorData, TactileGraspController::fingers_num> SensorArray;
typedef std::array<double, TactileGraspController::tactiles_num> PressureArray;
typedef std::array<std::pair<double, double>, TactileGraspController::tactiles_num> TactileXYArray;

class TactileSensorProcessor {
public:
  TactileSensorProcessor(const rclcpp::Logger& logger, const rclcpp::Clock::SharedPtr& clock);

  void init_tactiles();

  // pressure parse/update
  bool check_msg(const HandPressuresPtr msg) const;

  SensorArray parse_sensors(const HandPressuresPtr msg) const;

  void update_pressure(FingerArray& fingers, bool& baseline, const SensorArray& sensors);

  CopInfo calc_cop(int finger_idx, const PressureArray& p) const;

  std::optional<CorrectionDecision> pick_correction(const CopInfo& info) const;

private:
  double clamp(double v, double min_v, double max_v) const;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  TactileXYArray tactile_xy_{};

  bool baseline_{false};

  double tactile_x_{0.02}; // 2cm
  double tactile_y_{0.02}; // 2cm
  int baseline_sample_count_{30};

  double ema_alpha_{0.2}; // 이전 force 적용 비율

  double min_force_for_correction_{10.0};
  double y_center_threshold_{0.20}; // dead-zone(0~1)    : org 0.35
  double cost_threshold_{0.10};     // correction 시작 최소 cost
  double x_center_threshold_{0.50}; // top down    : 70 almost ignore    best : 60
};

} // namespace robotis_hand_tactile