#pragma once

#include "tactile_grasp_controller.hpp"

namespace robotis_hand_tactile {

typedef TactileGraspController Controller;
typedef TactileGraspController::CorrectionPlan CorrectionPlan;
typedef TactileGraspController::CorrectionType CorrectionType;
typedef TactileGraspController::HoldCorrectionStage HoldCorrectionStage;

class TactileCorrectionPlanner {
public:
  explicit TactileCorrectionPlanner(Controller& controller);

  void start_correction(int finger_idx);
  void apply_correction(int finger_idx);

  bool run_correction(int finger_idx, CorrectionPlan& plan);
  bool is_at_joint_limit(int finger_idx, CorrectionType type) const;
  bool is_y_correction(CorrectionType type) const;
  bool is_x_correction(CorrectionType type) const;
  bool correction_blocked(HoldCorrectionStage stage) const;
  std::string correction_str(CorrectionType t) const;

private:
  Controller& controller_;
};

} // namespace robotis_hand_tactile