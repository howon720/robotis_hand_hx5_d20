#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <utility>

class ReactiveForceNode : public rclcpp::Node
{
public:
  ReactiveForceNode();

private:
  void forceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void graspStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

  double mean(const std::deque<double> & data) const;
  double variation(const std::deque<double> & data) const;
  double slope(const std::deque<double> & data) const;

  void publishReleaseTrajectory(
    const std::vector<std::pair<std::string, int>> & release_fingers);

private:
  std::string force_topic_;
  std::string joint_state_topic_;
  std::string traj_topic_;

  int history_len_;
  int startup_ignore_steps_;
  int cooldown_steps_default_;
  double traj_time_;

  double joint_min_;
  double joint_max_;

  double delta_threshold_;
  double variation_threshold_;
  double min_contact_force_;

  std::vector<std::string> finger_names_;

  std::map<std::string, std::deque<double>> force_history_;
  std::map<std::string, int> cooldown_counter_;
  std::map<std::string, double> current_joint_positions_;

  // region 1,2 용
  std::map<std::string, std::vector<std::pair<std::string, double>>> joint_release_step_;

  // region 3,4 용
  std::map<std::string, std::pair<std::string, double>> joint_region34_step_;

  int step_count_{0};

  int grasp_state_{0};
  int post_grasp_ignore_steps_{50};
  int post_grasp_ignore_counter_{0};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
};