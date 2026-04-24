#include "param.h"

namespace robotis_hand_tactile {

void declare_params(rclcpp::Node* node) {
  // common
  node->declare_parameter<double>("control_hz", 20.0);
  node->declare_parameter<double>("trajectory_dt", 0.05);
  node->declare_parameter<double>("close_step", 0.01);
  node->declare_parameter<double>("contact_threshold", 30.0);
  node->declare_parameter<double>("thumb_contact_ratio", 2.0);
  node->declare_parameter<std::vector<int64_t>>("un_use_finger", std::vector<int64_t>{});

  // tactile_sensor
  node->declare_parameter<double>("x_center", 0.5);
  node->declare_parameter<double>("y_center", 0.2);
  node->declare_parameter<double>("min_force_correction", 10.0);
  node->declare_parameter<double>("cost_thres", 0.1);

  // hold
  node->declare_parameter<double>("reactive_force", 1.2);

  // optimize
  node->declare_parameter<double>("feedback_max_delta", 0.01);
  node->declare_parameter<double>("regrasp_force", 3.0);
}

Params load_params(rclcpp::Node* node) {
  Params p;

  // common
  node->get_parameter("control_hz", p.control_hz);
  node->get_parameter("trajectory_dt", p.trajectory_dt);
  node->get_parameter("close_step", p.close_step);
  node->get_parameter("contact_threshold", p.contact_threshold);
  node->get_parameter("thumb_contact_ratio", p.thumb_contact_ratio);

  std::vector<int64_t> un_use_finger_tmp{};
  node->get_parameter("un_use_finger", un_use_finger_tmp);
  p.un_use_finger.clear();
  for (const auto value : un_use_finger_tmp) {
    if (value == 0) {
      continue; // NONE
    }
    const int finger_idx = static_cast<int>(value - 1);
    if (finger_idx >= 0 && finger_idx < 5) {
      p.un_use_finger.push_back(finger_idx);
    }
  }

  // tactile_sensor
  node->get_parameter("x_center", p.x_center);
  node->get_parameter("y_center", p.y_center);
  node->get_parameter("min_force_correction", p.min_force_correction);
  node->get_parameter("cost_thres", p.cost_thres);

  // hold
  node->get_parameter("reactive_force", p.reactive_force);

  // optimize
  node->get_parameter("feedback_max_delta", p.feedback_max_delta);
  node->get_parameter("regrasp_force", p.regrasp_force);

  return p;
}

} // namespace robotis_hand_tactile