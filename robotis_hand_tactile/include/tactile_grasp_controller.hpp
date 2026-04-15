#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hx5d20_struct.h"
#include "finger_ik_solver.hpp"

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robotis_hand_tactile {

class TactileCorrectionPlanner;

class TactileGraspController : public rclcpp::Node {
public:
  static constexpr int fingers_num = 5;
  static constexpr int tactiles_num = 9;

  typedef std::array<double, tactiles_num> PressureArray;
  typedef std::array<double, 4> JointValueArray;

  enum class State {
    IDLE,
    CLOSE,
    HOLD
  };

  enum class CorrectionType {
    NONE,
    X_TOP,
    X_BOT,
    Y_LEFT,
    Y_RIGHT
  };

  enum class HoldCorrectionStage {
    Y_FIRST,
    X_SECOND
  };

  struct CopInfo {
    PressureArray pressure{};
    double total_force{0.0};

    double cop_x{0.0};
    double cop_y{0.0};

    double top_sum{0.0};
    double mid_sum{0.0};
    double bot_sum{0.0};

    double left_sum{0.0};
    double center_sum{0.0};
    double right_sum{0.0};

    double top_x_bias{0.0};
    double mid_x_bias{0.0};
    double bot_x_bias{0.0};

    // normalized CoP ratio (-1 ~ 1)
    double cop_x_ratio{0.0};
    double cop_y_ratio{0.0};

    // lateral correction cost
    double y_left_cost{0.0};
    double y_right_cost{0.0};
    double x_top_cost{0.0};
    double x_bot_cost{0.0};
  };

  struct Finger {
    std::string name;
    std::array<std::string, 4> joint_names{};
    JointValueArray joint_min{};
    JointValueArray joint_max{};
    JointValueArray current_joint_targets{};

    bool contact_detected{false};

    PressureArray baseline_sum_tactiles{};
    PressureArray baseline_tactiles{};
    PressureArray ema_tactiles{};
    int baseline_samples{0};

    double filtered_force{0.0};
    CopInfo cop{};
  };

  struct CorrectionPlan {
    bool active{false};
    CorrectionType type{CorrectionType::NONE};
    int phase{0};
    int ticks_remaining{0};
    double cost{0.0};
  };

  struct CorrectionDecision {
    CorrectionType type{CorrectionType::NONE};
    double cost{0.0};
  };

  explicit TactileGraspController(const std::string& node_name = "tactile_grasp_controller");
  virtual ~TactileGraspController();

  // init
  void init_fingers();
  void init_joints();

  // control loop
  void handle_idle();
  void handle_close();
  void handle_hold();

  void set_desired_force();
  void regulate_grasp_force(int finger_idx);

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

  bool get_target(const std::string& joint_name, double& target) const;
  double get_joint_pos(const std::string& joint_name) const;
  double get_open_pos(const std::string& joint_name) const;
  double clamp(double v, double min_v, double max_v) const;

  double finger_contact_threshold(int finger_idx) const;
  double finger_step_scale(int finger_idx) const;
  double max_filtered_force() const;

  bool correction_ik(int finger_idx, bool forward_y);
  std::array<double, 3> get_planar_q(int finger_idx) const;
  void set_planar_q(int finger_idx, const std::array<double, 3>& q);

  bool all_finger_contacted() const;

protected:
  virtual void publish_traj() = 0;
  virtual std::optional<CorrectionDecision> pick_correction(const CopInfo& info) const = 0;

protected:
  friend class TactileCorrectionPlanner;

  std::unique_ptr<FingerPlanarIk> finger_planar_ik_;
  std::unique_ptr<TactileCorrectionPlanner> correction_planner_;

  std::array<Finger, fingers_num> fingers_{};
  std::array<CorrectionPlan, fingers_num> correction_plans_{};
  std::array<double, fingers_num> desired_force_{};

  std::vector<std::string> hand_joint_names_;
  std::vector<double> init_positions_;
  std::map<std::string, double> curr_joint_;

  State state_{State::IDLE};
  HoldCorrectionStage hold_correction_stage_{HoldCorrectionStage::Y_FIRST};

  bool joint_received_{false};

  // control timing
  double control_hz_{20.0}; // handle_hold rate   50ms 0.05sec
  double trajectory_dt_{0.05};

  double close_step_{0.01};
  // cylinder_tape : 70(50) , dynamixel_box : 40 , tennisball : 30  , papercup : 10
  double contact_threshold_{30}; // threshold   use : 30

  // force control
  double force_kp_{0.002}; // force error 값 계수
  double deadband_L{5.0};  // 손떨림방지 : 이거 없어도 되는지 확인
  double deadband_H{5.0};
  double feedback_max_delta_{0.01}; // feedback max step

  // correction
  int phase_step_{4};
  double y_corr_step_{0.03};  // CoP X Y 사용 step
  double shift_step_{0.02};   // org : 0.01
  double regrasp_step_{0.02}; // corr 중 regrasp

  // finger scaling
  double min_step_scale_{0.3};
  double max_step_scale_{1.0};

  // special ratios
  double thumb_contact_ratio_{2.0}; // thumb contact = other finger threshold 2x
  double regrasp_force_ratio_{1.5}; // 재그립 완료 기준 = 평소 threshold의 1.5배
};

} // namespace robotis_hand_tactile