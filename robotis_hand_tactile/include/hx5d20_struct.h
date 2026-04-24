#pragma once

#include <array>
#include <string>
#include <vector>

namespace robotis_hand_tactile {

// Number of fingers, tactile cells, and joints.
constexpr int fingers_num = 5;
constexpr int tactiles_num = 9;
constexpr int joints_per_finger = 4;
typedef std::array<double, tactiles_num> PressureArray;

// Correction direction based on tactile CoP error.
enum class CorrectionType {
  NONE,
  Y_TOP,
  Y_BOT,
  X_LEFT,
  X_RIGHT
};
enum class HoldCorrectionStage {
  X_FIRST,
  Y_SECOND
};

// Tactile center-of-pressure information.
struct CopInfo {
  std::array<double, tactiles_num> pressure{};
  double total_force = 0.0;

  double cop_y = 0.0;
  double cop_x = 0.0;

  double cop_y_ratio = 0.0;
  double cop_x_ratio = 0.0;

  double x_left_cost = 0.0;
  double x_right_cost = 0.0;
  double y_top_cost = 0.0;
  double y_bot_cost = 0.0;
};

struct CorrectionDecision {
  CorrectionType type{CorrectionType::NONE};
  double cost = 0.0;
};

struct CorrectionPlan {
  bool active = false;
  CorrectionType type{CorrectionType::NONE};
  int phase = 0;
  int ticks_remaining = 0;
  double cost = 0.0;
};
typedef std::array<CorrectionPlan, fingers_num> CorrectionPlanArray;

struct FingerData {
  std::string name;

  std::array<std::string, joints_per_finger> joint_names{};
  std::array<double, joints_per_finger> joint_min{};
  std::array<double, joints_per_finger> joint_max{};
  std::array<double, joints_per_finger> current_joint_targets{};

  bool contact_detected{false};

  std::array<double, tactiles_num> baseline_sum_tactiles{};
  std::array<double, tactiles_num> baseline_tactiles{};
  std::array<double, tactiles_num> ema_tactiles{};
  int baseline_samples = 0;

  double filtered_force = 0.0;
  CopInfo cop{};
};
typedef std::array<FingerData, fingers_num> FingerArray;

struct Hx5d20SensorData {
  std::string name;
  std::array<std::string, tactiles_num> labels{};
  std::array<double, tactiles_num> values{};
};
typedef std::array<Hx5d20SensorData, fingers_num> SensorArray;

} // namespace robotis_hand_tactile