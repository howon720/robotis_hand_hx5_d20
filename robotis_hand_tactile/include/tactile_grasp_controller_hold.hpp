#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"

#include "hx5d20_struct.h"
#include "tactile_sensor.hpp"

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace robotis_hand_tactile_hold {

typedef robotis_interfaces::msg::HandPressures HandPressuresMsg;
typedef robotis_interfaces::msg::HandPressures::SharedPtr HandPressuresPtr;
typedef sensor_msgs::msg::JointState JointStateMsg;
typedef sensor_msgs::msg::JointState::SharedPtr JointStatePtr;
typedef std_msgs::msg::Int32 Int32Msg;
typedef std_msgs::msg::Int32::SharedPtr Int32Ptr;
typedef trajectory_msgs::msg::JointTrajectory JointTrajectoryMsg;
typedef robotis_hand_tactile::FingerArray FingerArrayMsg;

class TactileGraspController : public rclcpp::Node {
public:
  static constexpr int fingers_num = robotis_hand_tactile::fingers_num;

  enum class State {
    IDLE,
    CLOSE,
    HOLD
  };

  TactileGraspController();

private:
  void pressure_callback(const HandPressuresPtr msg);
  void joint_state_callback(const JointStatePtr msg);
  void grasp_state_callback(const Int32Ptr msg);

  void control_loop();

  void handle_idle();
  void handle_close();
  void handle_hold();

  void reset_grasp();
  void set_desired_force();
  void publish_traj();
  void sync_targets();

  bool all_contacted() const;
  bool get_target(const std::string& joint_name, double& target) const;

  double get_open_pos(const std::string& joint_name) const;
  double apply_deadband(double error) const;
  double clamp(double value, double min_v, double max_v) const;
  double get_joint_pos(const std::string& joint_name) const;
  double finger_contact_threshold(int finger_idx) const;

  // not use finger
  bool unused_finger(int finger_idx) const;
  void close_unused_finger();

  std::string state_to_string(State s) const;

private:
  rclcpp::Subscription<HandPressuresMsg>::SharedPtr pressure_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<Int32Msg>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<JointTrajectoryMsg>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr unused_finger_timer_;

  robotis_hand_tactile::TactileSensor tactile_sensor_;

  std::mutex mutex_;

  FingerArrayMsg fingers_{};

  std::array<double, fingers_num> contact_force_{};
  std::array<double, fingers_num> desired_force_{};
  std::array<double, fingers_num> prev_filtered_force_{};

  std::vector<std::string> hand_joint_names_;
  std::vector<double> init_positions_;
  std::map<std::string, double> curr_joint_;

  State state_{State::IDLE};

  bool joint_state_received_{false};
  bool baseline_{false};

  double control_hz_{20.0};
  double trajectory_dt_{0.05};

  double close_step_{0.03};
  double contact_threshold_{20.0};  // org : 10.0    //pinch : 3.0
  double thumb_contact_ratio_{2.0}; // org : 2.8     // pinch : 1.0

  double reactive_force_scale_{1.0}; // org : 1.0

  double deadband_L{-5.0}; // 오차 : 손떨림 보정
  double deadband_H{5.0};
  double kf_{0.002};
  double reactive_step_{0.01};

  // thumb: 0, index: 1, middle: 2, ring: 3, little: 4
  std::vector<int> not_use_fingers_{3, 4};
};

} // namespace robotis_hand_tactile_hold