#pragma once

#include <string>

#include "hx5d20_struct.h"

namespace robotis_hand_tactile {

class TactileGraspController;

/**
 * @brief Plans and applies tactile CoP-based correction motions.
 */
class TactileCorrectionPlanner {
public:
  explicit TactileCorrectionPlanner(TactileGraspController& controller);

  /**
   * @brief Start a correction plan if tactile CoP correction is required.
   */
  void start_correction(int finger_idx);

  /**
   * @brief Apply the active correction plan for the selected finger.
   */
  void apply_correction(int finger_idx);

  /**
   * @brief Run one step of the active correction plan.
   */
  bool run_correction(int finger_idx, CorrectionPlan& plan);

  /**
   * @brief Run one step of lateral x-axis correction motion.
   */
  bool run_x_correction(int finger_idx, CorrectionPlan& plan, int direction);

  /**
   * @brief Run one step of y-axis fingertip correction using planar IK.
   */
  bool run_y_correction(int finger_idx, CorrectionPlan& plan, bool forward_y);

  /**
   * @brief Check whether the correction motion can move within joint limits.
   */
  bool is_at_joint_limit(int finger_idx, CorrectionType type) const;

  /**
   * @brief Check whether the correction type belongs to y-direction correction.
   */
  bool is_y_correction(CorrectionType type) const;

  /**
   * @brief Check whether the correction type belongs to x-direction correction.
   */
  bool is_x_correction(CorrectionType type) const;

  /**
   * @brief Check whether all corrections in the current stage are blocked.
   */
  bool correction_blocked(HoldCorrectionStage stage) const;

  /**
   * @brief Convert correction type to string.
   */
  std::string correction_str(CorrectionType type) const;

private:
  // Controller reference used to access finger states and motion utilities.
  TactileGraspController& controller_;
};

} // namespace robotis_hand_tactile
