#pragma once

#include <array>
#include <map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "tactile_grasp_controller.hpp"

namespace robotis_hand_tactile {

class TactileGraspControllerNode : public rclcpp::Node
{
public:
  TactileGraspControllerNode();

  bool check_msg(const robotis_interfaces::msg::HandPressures::SharedPtr msg) const;

  std::array<Hx5d20SensorData, TactileGraspController::k_num_fingers> 
  parse_sensors(const robotis_interfaces::msg::HandPressures::SharedPtr msg) const;

  void on_pressure(
    const robotis_interfaces::msg::HandPressures::SharedPtr msg);

  void on_joint_state(
    const sensor_msgs::msg::JointState::SharedPtr msg);

  void on_grasp_state(
    const std_msgs::msg::Int32::SharedPtr msg);

  // main loop
  void control_loop();

  void publish_traj();

private:
  std::mutex mutex_;

  TactileGraspController controller_;

  rclcpp::Subscription<robotis_interfaces::msg::HandPressures>::SharedPtr pressure_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double control_rate_hz_{20.0};
  double trajectory_dt_{0.05};

  bool baseline_logged_{false};
};

}  // namespace robotis_hand_tactile