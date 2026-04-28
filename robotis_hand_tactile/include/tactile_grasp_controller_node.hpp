#pragma once

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "tactile_grasp_controller.hpp"
#include "tactile_sensor.hpp"

namespace robotis_hand_tactile {

typedef robotis_interfaces::msg::HandPressures HandPressuresMsg;
typedef robotis_interfaces::msg::HandPressures::SharedPtr HandPressuresPtr;
typedef sensor_msgs::msg::JointState JointStateMsg;
typedef sensor_msgs::msg::JointState::SharedPtr JointStatePtr;
typedef std_msgs::msg::Int32 Int32Msg;
typedef std_msgs::msg::Int32::SharedPtr Int32Ptr;
typedef trajectory_msgs::msg::JointTrajectory JointTrajectoryMsg;
typedef trajectory_msgs::msg::JointTrajectoryPoint JointTrajectoryPointMsg;

/**
 * @brief ROS 2 node wrapper for tactile CoP-based grasp control.
 */
class TactileGraspControllerNode : public TactileGraspController {
public:
  TactileGraspControllerNode();

private:
  /**
   * @brief Handle tactile pressure message.
   */
  void on_pressure(const HandPressuresPtr msg);

  /**
   * @brief Handle joint state message.
   */
  void on_joint_state(const JointStatePtr msg);

  /**
   * @brief Handle grasp state command.
   */
  void on_grasp_state(const Int32Ptr msg);

  /**
   * @brief Main controller loop.
   */
  void control_loop();

  /**
   * @brief Publish current joint targets as JointTrajectory.
   */
  void publish_traj() override;

  /**
   * @brief Select correction direction using tactile sensor CoP information.
   */
  std::optional<CorrectionDecision> pick_correction(const CopInfo& info) const override;

private:
  // Thread lock
  std::mutex mutex_;

  // ROS interfaces
  rclcpp::Subscription<HandPressuresMsg>::SharedPtr pressure_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<Int32Msg>::SharedPtr grasp_state_sub_;
  rclcpp::Publisher<JointTrajectoryMsg>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr unused_finger_timer_;

  // Tactile processing
  TactileSensor tactile_sensor_;
  bool baseline_ = false;
};

} // namespace robotis_hand_tactile
