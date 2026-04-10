#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "hx5d20_struct.h"
#include "finger_ik_solver.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace robotis_hand_tactile
{

class TactileGraspController : public rclcpp::Node
{
public:
  TactileGraspController();

private:
  static constexpr int fingers_num = 5;
  static constexpr int tactiles_num = 9;

  enum class State
  {
    IDLE,
    CLOSE,
    HOLD,
    OPEN
  };

  enum class CorrectionType
  {
    NONE,
    X_TOP,
    X_BOT,
    Y_LEFT,
    Y_RIGHT
  };

  struct CopInfo
  {
    std::array<double, tactiles_num> pressure{};
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

  struct Finger
  {
    std::string name;
    std::array<std::string, 4> joint_names{};
    std::array<double, 4> joint_min{};
    std::array<double, 4> joint_max{};
    std::array<double, 4> current_joint_targets{};

    bool contact_detected{false};

    std::array<double, tactiles_num> baseline_sum_tactiles{};
    std::array<double, tactiles_num> baseline_tactiles{};
    std::array<double, tactiles_num> ema_tactiles{};
    int baseline_samples{0};

    double filtered_force{0.0};
    CopInfo cop{};

    bool regrasp_mode{false};
    int regrasp_stable_ticks{0};

    // little 0.2 regrasp
    double prev_joint1_target{0.0};
    double joint1_motion_accum{0.0};
  };

  struct CorrectionPlan
  {
    bool active{false};
    CorrectionType type{CorrectionType::NONE};
    int phase{0};
    int ticks_remaining{0};
    double cost{0.0};
  };

  struct CorrectionDecision
  {
    CorrectionType type{CorrectionType::NONE};
    double cost{0.0};
  };

private:
  // init
  void init_fingers();
  void init_joints();
  void init_tactiles();

  // callbacks
  void on_pressure(const robotis_interfaces::msg::HandPressures::SharedPtr msg);
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_grasp_state(const std_msgs::msg::Int32::SharedPtr msg);

  // pressure parse/update
  bool check_msg(const robotis_interfaces::msg::HandPressures::SharedPtr msg) const;
  std::array<Hx5d20SensorData, fingers_num> parse_sensors(const robotis_interfaces::msg::HandPressures::SharedPtr msg) const;
  void update_pressure(const std::array<Hx5d20SensorData, fingers_num> & sensors);

  // tactile analysis
  CopInfo calc_cop(int finger_idx, const std::array<double, tactiles_num> & p) const;
  std::optional<CorrectionDecision> pick_correction(const CopInfo & info) const;

  // control loop
  void control_loop();
  void handle_idle();
  void handle_close();
  void handle_hold();
  void handle_open();

  void set_desired_force();
  void regulate_grasp_force(int finger_idx);

  void start_correction(int finger_idx);
  void apply_correction(int finger_idx);
  bool run_correction(int finger_idx, CorrectionPlan & plan);

  void grasp_step_hold(int finger_idx, double step);
  void release_step_hold(int finger_idx, double step);

  void apply_ratio_step(int finger_idx, const std::array<int, 3> & local_joint_ids,
                        const std::array<double, 3> & ratios, double signed_step);

  void shift_joint1(int finger_idx, double delta);
  void shift_thumb_y(int finger_idx, double delta1, double delta2);
  void grasp_234(int finger_idx, double step);
  void release_234(int finger_idx, double step);
  void mixed_bot_step(int finger_idx, double step);

  // helpers
  void reset_grasp();
  void sync_targets();
  bool all_contacted() const;

  void publish_traj();
  bool get_target(const std::string & joint_name, double & target) const;
  double get_joint_pos(const std::string & joint_name) const;
  double get_open_pos(const std::string & joint_name) const;
  double clamp(double v, double min_v, double max_v) const;

  std::string state_str(State s) const;
  std::string correction_str(CorrectionType t) const;

  // force 분리
  bool need_regrasp(int finger_idx) const;
  bool update_regrasp(int finger_idx);

  // each finger vel
  double finger_step_scale(int finger_idx) const;
  double max_filtered_force() const;

  // thumb & little
  double finger_contact_threshold(int finger_idx) const;
  void update_little_joint1();
  bool TL_regrasp_flag() const;
  bool TL_regrasp_done() const;
  void reset_TL_regrasp();

  // blocked min_max
  bool can_run_correction_step(int finger_idx, CorrectionType type) const;
  bool all_corrections_blocked() const;

  // ik
  bool apply_x_correction_by_ik(int finger_idx, bool forward_y, double cost);
  std::array<double, 3> get_planar_q(int finger_idx) const;
  void set_planar_q(int finger_idx, const std::array<double, 3> & q);

private:
  std::mutex mutex_;

  rclcpp::Subscription<robotis_interfaces::msg::HandPressures>::SharedPtr pressure_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<FingerPlanarIk> finger_planar_ik_;

  std::array<Finger, fingers_num> fingers_{};
  std::array<CorrectionPlan, fingers_num> correction_plans_{};
  std::array<double, fingers_num> desired_force_{};

  std::vector<std::string> all_hand_joint_names_;
  std::vector<double> open_reference_positions_;
  std::map<std::string, double> joint_position_map_;

  std::array<std::pair<double, double>, tactiles_num> tactile_xy_{};

  State state_{State::IDLE};

  bool joint_received_{false};
  bool baseline_{false};

  // params
  double control_rate_hz_{20.0};   // handle_hold rate   50ms 0.05sec
  double trajectory_dt_{0.05};

  double tactile_x_{0.02};  // 2cm
  double tactile_y_{0.02};  // 2cm

  int baseline_sample_count_{30};

  // 
  double ema_alpha_{0.2};

  double close_step_{0.01};
  double open_step_{0.02};

  // cylinder_tape : 70(50) , dynamixel_box : 40 , tennisball : 30  , papercup : 10
  double contact_threshold_{40};   // threshold

  double force_kp_{0.002};
  double deadband_L{5.0};
  double deadband_H{5.0};
  double feedback_max_delta_{0.01};  // feedback max step

  double min_force_for_correction_{10.0};
  double X_diff_threshold_{5000.0};
  double Y_diff_threshold_{5.0};      // 이것도 contact_threshold 에 비례해서 나와야 함.    : 이렇게 정해져 있는 값보다는 cost 기반으로 바꾸긴 해야할 듯

  // cost
  double y_center_ratio_threshold_{0.30};   // dead-zone(0~1)    : org 0.35
  double y_cost_trigger_threshold_{0.10};   // correction 시작 최소 cost
  double x_center_ratio_threshold_{0.60};   // top down    : 70 almost ignore
  double x_cost_trigger_threshold_{0.10};

  int phase_step_{4};

  double release_step_base_{0.03};   // CoP X Y 사용 step
  double grasp_step_base_{0.03};
  double joint1_shift_step_{0.01};

  // regrasp 분리
  double regrasp_trigger_ratio_{0.7};
  int regrasp_stable_count_{3};     // 몇 tick 연속 안정되면 종료
  double regrasp_step_{0.02}; 

  // finger 마다 속도 다르게
  double min_step_scale_{0.3};
  double max_step_scale_{1.0};

  // little 0.2 regrasp
  bool TL_regrasp_mode_{false};
  double thumb_contact_ratio_{2.0};          // thumb contact = other finger threshold 2x
  double regrasp_force_ratio_{1.5};          // 재그립 완료 기준 = 평소 threshold의 1.5배
  double little_regrasp_delta_{1.0};         // little joint1 누적 변화량 기준
  double TL_grasp_step_{0.01}; 

  // IK
  double ik_y_shift_base_{0.003};     // 3 mm
  double ik_min_cost_scale_{0.3};
};

}  // namespace robotis_hand_tactile
