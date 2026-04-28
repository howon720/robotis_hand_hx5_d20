#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robotis_hand_tactile {

// Parameters used by tactile grasp controllers.
struct Params {
  // Common control parameters
  double control_hz = 20.0;
  double trajectory_dt = 0.05;
  double close_step = 0.01;
  double contact_threshold = 30.0;
  double thumb_contact_ratio = 2.0;
  std::vector<int> un_use_finger;

  // Tactile CoP correction parameters
  double y_center = 0.5;
  double x_center = 0.2;
  double min_force_correction = 10.0;
  double cost_thres = 0.1;

  // Force maintenance parameters
  double reactive_force = 1.2;

  // Optimization grasping parameters
  double feedback_max_delta = 0.01;
  double regrasp_force = 3.0;
};

/**
 * @brief Declare ROS 2 parameters.
 */
void declare_params(rclcpp::Node* node);
/**
 * @brief Load ROS 2 parameters.
 */
Params load_params(rclcpp::Node* node);

} // namespace robotis_hand_tactile
