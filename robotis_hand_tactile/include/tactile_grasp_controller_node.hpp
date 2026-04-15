#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"

#include "tactile_grasp_controller.hpp"
#include "tactile_sensor_processor.hpp"

#include <mutex>

namespace robotis_hand_tactile {

typedef robotis_interfaces::msg::HandPressures HandPressuresMsg;
typedef robotis_interfaces::msg::HandPressures::SharedPtr HandPressuresPtr;
typedef sensor_msgs::msg::JointState JointStateMsg;
typedef sensor_msgs::msg::JointState::SharedPtr JointStatePtr;
typedef std_msgs::msg::Int32 Int32Msg;
typedef std_msgs::msg::Int32::SharedPtr Int32Ptr;
typedef trajectory_msgs::msg::JointTrajectory JointTrajectoryMsg;
typedef trajectory_msgs::msg::JointTrajectoryPoint JointTrajectoryPointMsg;

typedef TactileGraspController::CorrectionDecision CorrectionDecision;
typedef TactileGraspController::CopInfo CopInfo;

class TactileGraspControllerNode : public TactileGraspController {
public:
  TactileGraspControllerNode();

private:
  void on_pressure(const HandPressuresPtr msg);
  void on_joint_state(const JointStatePtr msg);
  void on_grasp_state(const Int32Ptr msg);
  void control_loop();

  void publish_traj() override;
  std::optional<CorrectionDecision> pick_correction(const CopInfo& info) const override;

private:
  std::mutex mutex_;

  rclcpp::Subscription<HandPressuresMsg>::SharedPtr pressure_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<Int32Msg>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<JointTrajectoryMsg>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  TactileSensorProcessor tactile_sensor_processor_;

  bool baseline_ = false;
};

} // namespace robotis_hand_tactile