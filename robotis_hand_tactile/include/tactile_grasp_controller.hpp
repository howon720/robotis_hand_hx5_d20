#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "hx5d20_struct.h"

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
  static constexpr int k_num_fingers = 5;
  static constexpr int k_num_tactiles = 9;

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
    std::array<double, k_num_tactiles> pressure{};
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
  };

  struct Finger
  {
    std::string name;
    std::array<std::string, 4> joint_names{};
    std::array<double, 4> joint_min{};
    std::array<double, 4> joint_max{};
    std::array<double, 4> current_joint_targets{};

    bool contact_detected{false};

    std::array<double, k_num_tactiles> baseline_sum_tactiles{};
    std::array<double, k_num_tactiles> baseline_tactiles{};
    std::array<double, k_num_tactiles> ema_tactiles{};
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
  std::array<Hx5d20SensorData, k_num_fingers> parse_sensors(const robotis_interfaces::msg::HandPressures::SharedPtr msg) const;
  void update_pressure(const std::array<Hx5d20SensorData, k_num_fingers> & sensors);

  // tactile analysis
  CopInfo calc_cop(const std::array<double, k_num_tactiles> & p) const;
  std::optional<CorrectionType> pick_correction(const CopInfo & info) const;

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

private:
  std::mutex mutex_;

  rclcpp::Subscription<robotis_interfaces::msg::HandPressures>::SharedPtr pressure_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::array<Finger, k_num_fingers> fingers_{};
  std::array<CorrectionPlan, k_num_fingers> correction_plans_{};
  std::array<double, k_num_fingers> desired_force_{};

  std::vector<std::string> all_hand_joint_names_;
  std::vector<double> open_reference_positions_;
  std::map<std::string, double> joint_position_map_;

  std::array<std::pair<double, double>, k_num_tactiles> tactile_xy_{};

  State state_{State::IDLE};

  bool joint_received_{false};
  bool baseline_{false};

  // params
  double control_rate_hz_{20.0};   // handle_hold rate   50ms 0.05sec
  double trajectory_dt_{0.05};

  double tactile_x_{0.005};
  double tactile_y_{0.005};

  int baseline_sample_count_{30};

  // 
  double ema_alpha_{0.2};

  double close_step_{0.01};
  double open_step_{0.02};

  // 70(50) : 원통형 테이프, dynamixel box : 40 , tennisball : 30     
  double contact_threshold_{30.0};   // threshold

  double force_kp_{0.002};
  double deadband_L{5.0};
  double deadband_H{5.0};
  double feedback_max_delta_{0.01};  // feedback max step

  double min_force_for_correction_{20.0};
  double X_diff_threshold_{50.0};
  double Y_diff_threshold_{30.0};

  int phase_step_{4};

  double release_step_base_{0.03};   // CoP X Y 사용 step
  double grasp_step_base_{0.03};
  double joint1_shift_step_{0.01};

  // regrasp 분리
  double regrasp_trigger_ratio_{0.75};
  int regrasp_stable_count_{3};     // 몇 tick 연속 안정되면 종료
  double regrasp_step_{0.02}; 

  // finger 마다 속도 다르게
  double min_step_scale_{0.3};
  double max_step_scale_{1.0};

  // little 0.2 regrasp
  bool TL_regrasp_mode_{false};
  double thumb_contact_ratio_{2.0};          // thumb contact = other finger threshold 2x
  double regrasp_force_ratio_{1.5};          // 재그립 완료 기준 = 평소 threshold의 1.5배
  double little_regrasp_delta_{0.2};         // little joint1 누적 변화량 기준
  double TL_grasp_step_{0.01}; 
};

}  // namespace robotis_hand_tactile