#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <std_msgs/msg/int32.hpp>

#include <deque>
#include <unordered_map>
#include <vector>
#include <string>

class ReactiveForceNode : public rclcpp::Node
{
public:
  ReactiveForceNode();

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  // grasp_state sub 하는거
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_state_sub_;
  int grasp_state_ = 0;
  int post_grasp_ignore_steps_ = 100;
  int post_grasp_ignore_counter_ = 0;

  std::string force_topic_;
  std::string joint_state_topic_;
  std::string traj_topic_;

  std::vector<std::string> finger_names_;

  std::unordered_map<std::string, std::deque<double>> force_history_;
  std::unordered_map<std::string, int> cooldown_counter_;
  std::unordered_map<std::string, double> current_joint_positions_;

  std::unordered_map<std::string, double> delta_threshold_;
  std::unordered_map<std::string, double> variation_threshold_;
  std::unordered_map<std::string, double> min_contact_force_;

  // finger -> (joint_name -> release_step)
  std::unordered_map<std::string, std::unordered_map<std::string, double>> joint_release_step_;

  int history_len_;
  int startup_ignore_steps_;
  int cooldown_steps_default_;
  int step_count_;

  double traj_time_;
  double joint_min_;
  double joint_max_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void forceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  double mean(const std::deque<double> & data) const;
  double variation(const std::deque<double> & data) const;
  double slope(const std::deque<double> & data) const;

  void publishReleaseTrajectory(const std::vector<std::string> & release_fingers);

  // grasp 신호
  void graspStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
};