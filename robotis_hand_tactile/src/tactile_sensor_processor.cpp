#include "tactile_sensor_processor.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace robotis_hand_tactile {

TactileSensorProcessor::TactileSensorProcessor(const rclcpp::Logger& logger, const rclcpp::Clock::SharedPtr& clock)
    : logger_(logger), clock_(clock) {
  init_tactiles();
}

void TactileSensorProcessor::init_tactiles() {
  const double x_offset = tactile_x_ / 3.0;
  const double y_offset = tactile_y_ / 3.0;

  const std::array<double, 3> xs = {-x_offset, 0.0, x_offset};
  const std::array<double, 3> ys = {-y_offset, 0.0, y_offset};

  int idx = 0;
  for (double y : ys) {
    for (double x : xs) {
      tactile_xy_[idx++] = {x, y};
    }
  }
}

bool TactileSensorProcessor::check_msg(const HandPressuresPtr msg) const {
  if (msg->sensors.size() != fingers_num) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "sensors size mismatch: %zu", msg->sensors.size());
    return false;
  }

  for (size_t i = 0; i < msg->sensors.size(); ++i) {
    if (msg->sensors[i].pressure_names.size() != tactiles_num) {
      RCLCPP_WARN_THROTTLE(logger_,
                           *clock_,
                           2000,
                           "sensor[%zu] pressure_names size mismatch: %zu",
                           i,
                           msg->sensors[i].pressure_names.size());
      return false;
    }

    if (msg->sensors[i].pressure_values.size() != tactiles_num) {
      RCLCPP_WARN_THROTTLE(logger_,
                           *clock_,
                           2000,
                           "sensor[%zu] pressure_values size mismatch: %zu",
                           i,
                           msg->sensors[i].pressure_values.size());
      return false;
    }
  }

  return true;
}

SensorArray TactileSensorProcessor::parse_sensors(const HandPressuresPtr msg) const {
  SensorArray out{};

  for (size_t i = 0; i < fingers_num; ++i) {
    out[i].name = msg->sensors[i].sensor_name;

    for (size_t j = 0; j < tactiles_num; ++j) {
      out[i].labels[j] = msg->sensors[i].pressure_names[j];
      out[i].values[j] = static_cast<double>(msg->sensors[i].pressure_values[j]);
    }
  }

  return out;
}

bool TactileSensorProcessor::update_baseline(FingerArray& fingers, bool& baseline, const SensorArray& sensors) {
  if (baseline) {
    return false;
  }

  for (int f = 0; f < fingers_num; ++f) {
    for (int t = 0; t < tactiles_num; ++t) {
      fingers[f].baseline_sum_tactiles[t] += sensors[f].values[t];
    }
    fingers[f].baseline_samples++;
  }

  bool ready = true;
  for (int f = 0; f < fingers_num; ++f) {
    if (fingers[f].baseline_samples < baseline_sample_count_) {
      ready = false;
      break;
    }
  }

  if (!ready) {
    return true;
  }

  for (int f = 0; f < fingers_num; ++f) {
    const int count = std::max(1, fingers[f].baseline_samples);
    for (int t = 0; t < tactiles_num; ++t) {
      fingers[f].baseline_tactiles[t] = fingers[f].baseline_sum_tactiles[t] / static_cast<double>(count);
      fingers[f].ema_tactiles[t] = 0.0;
    }
  }

  baseline = true;
  RCLCPP_INFO(logger_, "Tactile-wise baseline ready.");
  return true;
}

PressureArray TactileSensorProcessor::filter_pressure(FingerData& finger, const Hx5d20SensorData& sensor) const {
  PressureArray filtered{};

  for (int t = 0; t < tactiles_num; ++t) {
    double v = sensor.values[t];
    v -= finger.baseline_tactiles[t];

    if (v < 0.0) { // baseline 보다 압력이 낮아질 시 0으로 고정
      v = 0.0;
    }

    finger.ema_tactiles[t] = (1.0 - ema_alpha_) * finger.ema_tactiles[t] + ema_alpha_ * v;

    filtered[t] = finger.ema_tactiles[t];
  }

  return filtered;
}

double TactileSensorProcessor::calc_total_force(const PressureArray& pressure) const {
  return std::accumulate(pressure.begin(), pressure.end(), 0.0);
}

void TactileSensorProcessor::update_finger_state(int finger_idx,
                                                 FingerData& finger,
                                                 const Hx5d20SensorData& sensor) const {
  const auto filtered = filter_pressure(finger, sensor);
  const double total_force = calc_total_force(filtered);

  finger.filtered_force = (1.0 - ema_alpha_) * finger.filtered_force + ema_alpha_ * total_force;

  finger.cop = calc_cop(finger_idx, filtered);
}

void TactileSensorProcessor::update_pressure(FingerArray& fingers, bool& baseline, const SensorArray& sensors) {
  if (update_baseline(fingers, baseline, sensors)) {
    return;
  }

  for (int f = 0; f < fingers_num; ++f) {
    update_finger_state(f, fingers[f], sensors[f]);
  }
}

// ==========================================
// CoP 계산
CopInfo TactileSensorProcessor::calc_cop(int finger_idx, const PressureArray& p) const {
  CopInfo info;
  info.pressure = p;
  info.total_force = std::accumulate(p.begin(), p.end(), 0.0);

  if (info.total_force <= 1e-9) {
    return info;
  }

  for (int i = 0; i < tactiles_num; ++i) {
    info.cop_x += p[i] * tactile_xy_[i].first;
    info.cop_y += p[i] * tactile_xy_[i].second;
  }
  info.cop_x /= info.total_force;
  info.cop_y /= info.total_force;

  info.top_sum = p[0] + p[1] + p[2];
  info.mid_sum = p[3] + p[4] + p[5];
  info.bot_sum = p[6] + p[7] + p[8];

  info.left_sum = p[0] + p[3] + p[6];
  info.center_sum = p[1] + p[4] + p[7];
  info.right_sum = p[2] + p[5] + p[8];

  info.top_x_bias = (-1.0 * p[0]) + (0.0 * p[1]) + (1.0 * p[2]);
  info.mid_x_bias = (-1.0 * p[3]) + (0.0 * p[4]) + (1.0 * p[5]);
  info.bot_x_bias = (-1.0 * p[6]) + (0.0 * p[7]) + (1.0 * p[8]);

  // normalize CoP to [-1, 1] using half size of tactile sensor   : 센서 중심 0, left,down : -1 , right,up: +1
  const double half_x = tactile_x_ * 0.5;
  const double half_y = tactile_y_ * 0.5;

  info.cop_x_ratio = clamp(info.cop_x / std::max(half_x, 1e-9), -1.0, 1.0);
  info.cop_y_ratio = clamp(info.cop_y / std::max(half_y, 1e-9), -1.0, 1.0);

  // Y_LEFT / Y_RIGHT decision is based on left-right CoP position
  // center area is wider and configurable by y_center_ratio_threshold_
  const double abs_x_ratio = std::fabs(info.cop_x_ratio);

  double y__ = y_center_threshold_ +
               ((finger_idx == 0) ? 0.45 : 0.0); // 일단은 thumb 무시하는 코드가 여기에 추가되어 있음. 이것도 수정 필요
  double x__ = x_center_threshold_ + ((finger_idx == 0) ? 0.45 : 0.0);

  if (abs_x_ratio > y__) {
    const double raw_cost_y = (abs_x_ratio - y__) / std::max(1.0 - y__, 1e-9);

    const double cost_y = clamp(raw_cost_y, 0.0, 1.0); // 정규화 [0,1]

    if (info.cop_x_ratio < 0.0) {
      info.y_left_cost = cost_y;
      info.y_right_cost = 0.0;
    } else {
      info.y_left_cost = 0.0;
      info.y_right_cost = cost_y;
    }
  }

  // X_TOP / X_BOT decision is based on top-bottom CoP position
  const double abs_y_ratio = std::fabs(info.cop_y_ratio);

  if (abs_y_ratio > x__) {
    const double raw_cost_x = (abs_y_ratio - x__) / std::max(1.0 - x__, 1e-9);

    const double cost_x = clamp(raw_cost_x, 0.0, 1.0);

    if (info.cop_y_ratio < 0.0) {
      info.x_top_cost = cost_x;
      info.x_bot_cost = 0.0;
    } else {
      info.x_top_cost = 0.0;
      info.x_bot_cost = cost_x;
    }
  }

  return info;
}
// ============================================

std::optional<CorrectionDecision> TactileSensorProcessor::pick_correction(const CopInfo& info) const {
  if (info.total_force < min_force_for_correction_) {
    return std::nullopt;
  }

  // X_TOP / X_BOT by CoP ratio-based cost
  const double cost_x = std::max(info.x_top_cost, info.x_bot_cost);
  if (cost_x >= cost_threshold_) {
    if (info.x_top_cost > info.x_bot_cost) {
      return CorrectionDecision{CorrectionType::X_TOP, info.x_top_cost};
    } else {
      return CorrectionDecision{CorrectionType::X_BOT, info.x_bot_cost};
    }
  }

  // Y_LEFT / Y_RIGHT by CoP ratio-based cost
  const double cost_y = std::max(info.y_left_cost, info.y_right_cost);
  if (cost_y >= cost_threshold_) {
    if (info.y_left_cost > info.y_right_cost) {
      return CorrectionDecision{CorrectionType::Y_LEFT, info.y_left_cost};
    } else {
      return CorrectionDecision{CorrectionType::Y_RIGHT, info.y_right_cost};
    }
  }

  return std::nullopt;
}

double TactileSensorProcessor::clamp(double v, double min_v, double max_v) const {
  return std::max(min_v, std::min(v, max_v));
}

} // namespace robotis_hand_tactile