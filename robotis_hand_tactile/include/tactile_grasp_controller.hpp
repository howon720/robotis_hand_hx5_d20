#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hx5d20_struct.h"
#include "hx5d20_init.hpp"
#include "finger_ik_solver.hpp"
#include "param.h"

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robotis_hand_tactile {
typedef std::array<double, tactiles_num> PressureArray;
typedef std::array<double, 4> JointValueArray;

class TactileCorrectionPlanner;

class TactileGraspController : public rclcpp::Node {
public:
  enum class State {
    IDLE,
    CLOSE,
    HOLD
  };

  /**
   * @brief Construct tactile grasp controller.
   */
  explicit TactileGraspController(const std::string& node_name = "tactile_grasp_controller");
  virtual ~TactileGraspController();

  /**
   * @brief Initialize finger information.
   */
  void init_fingers();

  /**
   * @brief Initialize joint target values.
   */
  void init_joints();

  // State handlers
  void handle_idle();
  void handle_close();
  void handle_hold();

  // Force control
  void set_desired_force();
  void regulate_grasp_force(int finger_idx);

  // Joint command utilities
  void grasp_step_hold(int finger_idx, double step);
  void release_step_hold(int finger_idx, double step);
  void apply_ratio_step(int finger_idx,
                        const std::array<int, 3>& local_joint_ids,
                        const std::array<double, 3>& ratios,
                        double signed_step);
  void shift_joint1(int finger_idx, double delta);
  void shift_thumb_y(int finger_idx, double delta1, double delta2);
  void grasp_234(int finger_idx, double step);
  void release_234(int finger_idx, double step);
  void reset_grasp();
  void sync_targets();
  bool all_contacted() const;
  bool all_finger_contacted() const;
  bool get_target(const std::string& joint_name, double& target) const;
  double get_joint_pos(const std::string& joint_name) const;
  double get_open_pos(const std::string& joint_name) const;
  double clamp(double v, double min_v, double max_v) const;

  double finger_contact_threshold(int finger_idx) const;
  double finger_step_scale(int finger_idx) const;
  double max_filtered_force() const;

  // IK
  bool correction_ik(int finger_idx, bool forward_y);
  std::array<double, 3> get_planar_q(int finger_idx) const;
  void set_planar_q(int finger_idx, const std::array<double, 3>& q);

  // Unused finger handling
  bool unused_finger(int finger_idx) const;
  void close_unused_finger();

protected:
  virtual void publish_traj() = 0;
  virtual std::optional<CorrectionDecision> pick_correction(const CopInfo& info) const = 0;

protected:
  // Parameters
  friend class TactileCorrectionPlanner;
  robotis_hand_tactile::Params param;

  // IK and correction planner
  std::unique_ptr<FingerPlanarIk> finger_planar_ik_;
  std::unique_ptr<TactileCorrectionPlanner> correction_planner_;

  // Finger and joint states
  FingerArray fingers_;
  CorrectionPlanArray correction_plans_;
  std::array<double, fingers_num> desired_force_;

  std::vector<std::string> hand_joint_names_;
  std::vector<double> init_positions_;
  std::map<std::string, double> curr_joint_;

  // Controller state
  State state_{State::IDLE};
  HoldCorrectionStage hold_correction_stage_{HoldCorrectionStage::X_FIRST};

  bool joint_received_ = false;

  // Force control
  double force_kp_ = 0.002; // Force feedback gain
  double deadband = 5.0;    // Deadband for small force errors

  // CoP correction
  int phase_step_ = 4;
  double x_corr_step_ = 0.03;
  double shift_step_ = 0.02;
  double regrasp_step_ = 0.02;

  // Finger step scaling
  double min_step_scale_ = 0.3; // 이것도 없어도 되는지 확인
  double max_step_scale_ = 1.0;

  std::array<bool, fingers_num> y_ik_failed_{false, false, false, false, false};
};

} // namespace robotis_hand_tactile