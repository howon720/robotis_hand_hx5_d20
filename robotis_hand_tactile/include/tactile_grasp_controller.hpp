#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

namespace robotis_hand_tactile
{

class TactileGraspController : public rclcpp::Node
{
public:
  TactileGraspController();

private:
  static constexpr int k_num_fingers = 5;

  enum class State
  {
    IDLE,
    CLOSE,
    HOLD,
    OPEN
  };

  struct FingerConfig
  {
    std::string name;

    std::vector<std::string> joint_names;
    std::vector<double> weights;
    std::vector<double> current_joint_targets;

    std::vector<double> joint_min;
    std::vector<double> joint_max;

    double raw_force{0.0};
    double filtered_force{0.0};
    double baseline_force{0.0};

    bool contact_detected{false};
  };

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tactile_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  State state_{State::IDLE};

  std::array<FingerConfig, k_num_fingers> fingers_;

  std::vector<std::string> all_control_joint_names_;
  std::vector<std::string> all_hand_joint_names_;
  std::vector<double> open_reference_positions_;
  std::unordered_map<std::string, double> joint_position_map_;
  bool joint_state_received_{false};

  std::array<double, k_num_fingers> desired_force_{};
  std::array<double, k_num_fingers> contact_force_{};
  std::array<double, k_num_fingers> prev_filtered_force_{};

  double control_rate_hz_{20.0};
  double alpha_{0.2};
  double contact_threshold_{100000.0};  // contact 임계값
  double force_target_scale_{1.2};  // HOLD 할 때의 목표값  : contact 기준 1.2 sclae
  double kf_{0.002};
  double deadband_low_{-0.03};   // 오차 : 손떨림 보정
  double deadband_high_{0.03};
  double close_step_{0.01};  //0.01
  double open_step_{0.015};
  double max_delta_per_step_{0.01};
  double slip_drop_threshold_{0.25};
  double slip_boost_{0.005};

  bool use_baseline_{false};
  int baseline_sample_count_{30};
  int baseline_collected_count_{0};
  bool baseline_ready_{false};

  double trajectory_dt_{0.1};

  void declare_parameters();
  void load_parameters();
  void init_finger_configs();
  void init_joint_name_list();

  void tactile_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void grasp_state_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void control_loop();

  void handle_idle();
  void handle_close();
  void handle_hold();
  void handle_open();

  void reset_for_new_grasp();
  void set_desired_force_from_contact();
  void publish_trajectory();
  void sync_targets_from_joint_state();

  bool all_fingers_contacted() const;
  bool detect_slip(int finger_idx) const;
  bool get_mapped_joint_target(const std::string & joint_name, double & target) const;
  double get_open_reference_position(const std::string & joint_name) const;

  double apply_deadband(double error) const;
  double clamp(double value, double min_v, double max_v) const;
  double get_joint_position(const std::string & joint_name) const;

  std::string state_to_string(State s) const;
};

}  // namespace robotis_hand_tactile