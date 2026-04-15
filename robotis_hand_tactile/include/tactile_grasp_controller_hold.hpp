#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"

#include "hx5d20_struct.h"
#include "tactile_sensor_processor.hpp"

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace robotis_hand_tactile_hold {

class TactileGraspController : public rclcpp::Node {
public:
  static constexpr int fingers_num = robotis_hand_tactile::fingers_num;

  typedef robotis_interfaces::msg::HandPressures HandPressuresMsg;
  typedef robotis_interfaces::msg::HandPressures::SharedPtr HandPressuresPtr;

  typedef sensor_msgs::msg::JointState JointStateMsg;
  typedef sensor_msgs::msg::JointState::SharedPtr JointStatePtr;

  typedef std_msgs::msg::Int32 Int32Msg;
  typedef std_msgs::msg::Int32::SharedPtr Int32Ptr;

  typedef trajectory_msgs::msg::JointTrajectory JointTrajectoryMsg;
  typedef trajectory_msgs::msg::JointTrajectoryPoint JointTrajectoryPointMsg;

  typedef robotis_hand_tactile::FingerData FingerDataMsg;
  typedef robotis_hand_tactile::FingerArray FingerArrayMsg;

  enum class State {
    IDLE,
    CLOSE,
    HOLD
  };

  TactileGraspController();

private:
  void init_finger_configs();
  void init_joint_name_list();

  void pressure_callback(const HandPressuresPtr msg);
  void joint_state_callback(const JointStatePtr msg);
  void grasp_state_callback(const Int32Ptr msg);

  void control_loop();

  void handle_idle();
  void handle_close();
  void handle_hold();

  void reset_for_new_grasp();
  void set_desired_force_from_contact();
  void publish_trajectory();
  void sync_targets_from_joint_state();

  bool all_fingers_contacted() const;
  bool get_mapped_joint_target(const std::string& joint_name, double& target) const;

  double get_open_reference_position(const std::string& joint_name) const;
  double apply_deadband(double error) const;
  double clamp(double value, double min_v, double max_v) const;
  double get_joint_position(const std::string& joint_name) const;
  double finger_contact_threshold(int finger_idx) const;

  std::string state_to_string(State s) const;

private:
  rclcpp::Subscription<HandPressuresMsg>::SharedPtr pressure_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<Int32Msg>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<JointTrajectoryMsg>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  robotis_hand_tactile::TactileSensorProcessor tactile_sensor_processor_;

  FingerArrayMsg fingers_{};

  std::array<double, fingers_num> contact_force_{};
  std::array<double, fingers_num> desired_force_{};
  std::array<double, fingers_num> prev_filtered_force_{};

  std::vector<std::string> all_hand_joint_names_;
  std::vector<double> open_reference_positions_;

  std::map<std::string, double> joint_position_map_;

  State state_{State::IDLE};

  bool joint_state_received_{false};
  bool baseline_{false};

  double control_rate_hz_{20.0};
  double trajectory_dt_{0.05};

  double close_step_{0.01};
  double contact_threshold_{30.0};
  double thumb_contact_ratio_{2.0};

  double force_target_scale_{1.0};

  double deadband_low_{-5.0};
  double deadband_high_{5.0};
  double kf_{0.002};
  double max_delta_per_step_{0.01};
};

} // namespace robotis_hand_tactile_hold