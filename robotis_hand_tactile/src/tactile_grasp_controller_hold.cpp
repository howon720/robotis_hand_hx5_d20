#include "tactile_grasp_controller_hold.hpp"

#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace robotis_hand_tactile_hold {

TactileGraspController::TactileGraspController()
    : Node("tactile_grasp_controller"), tactile_sensor_processor_(this->get_logger(), this->get_clock()) {
  init_finger_configs();
  init_joint_name_list();

  pressure_sub_ = this->create_subscription<robotis_interfaces::msg::HandPressures>(
      "/right_hand/finger_pressures",
      10,
      std::bind(&TactileGraspController::pressure_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&TactileGraspController::joint_state_callback, this, std::placeholders::_1));

  grasp_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/grasp_state", 10, std::bind(&TactileGraspController::grasp_state_callback, this, std::placeholders::_1));

  traj_pub_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/right_hand_controller/joint_trajectory", 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                           std::bind(&TactileGraspController::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "TactileGraspController initialized.");
}

void TactileGraspController::init_finger_configs() {
  // thumb
  fingers_[0].name = "thumb";
  fingers_[0].joint_names = {"finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4"};
  fingers_[0].joint_min = {-1.5, -3.5, -1.5, -1.5};
  fingers_[0].joint_max = {1.5, 0.5, 1.5, 1.5};
  fingers_[0].current_joint_targets = {0.0, 0.0, 0.0, 0.0};

  // index
  fingers_[1].name = "index";
  fingers_[1].joint_names = {"finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8"};
  fingers_[1].joint_min = {-1.5, -1.5, -1.5, -1.5};
  fingers_[1].joint_max = {0.6, 1.5, 1.5, 1.5};
  fingers_[1].current_joint_targets = {0.0, 0.0, 0.0, 0.0};

  // middle
  fingers_[2].name = "middle";
  fingers_[2].joint_names = {"finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12"};
  fingers_[2].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[2].joint_max = {0.6, 1.5, 1.5, 1.5};
  fingers_[2].current_joint_targets = {0.0, 0.0, 0.0, 0.0};

  // ring
  fingers_[3].name = "ring";
  fingers_[3].joint_names = {"finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16"};
  fingers_[3].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[3].joint_max = {0.6, 1.5, 1.5, 1.5};
  fingers_[3].current_joint_targets = {0.0, 0.0, 0.0, 0.0};

  // little
  fingers_[4].name = "little";
  fingers_[4].joint_names = {"finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};
  fingers_[4].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[4].joint_max = {1.5, 1.5, 1.5, 1.5};
  fingers_[4].current_joint_targets = {0.0, 0.0, 0.0, 0.0};
}

void TactileGraspController::init_joint_name_list() {
  all_hand_joint_names_ = {"finger_r_joint1",  "finger_r_joint2",  "finger_r_joint3",  "finger_r_joint4",
                           "finger_r_joint5",  "finger_r_joint6",  "finger_r_joint7",  "finger_r_joint8",
                           "finger_r_joint9",  "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
                           "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
                           "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};

  open_reference_positions_ = {
      0.297, -1.792, 0.0, 0.0, // 1~4
      0.0,   0.8,    0.0, 0.0, // 5~8
      0.0,   0.8,    0.0, 0.0, // 9~12
      0.0,   0.8,    0.0, 0.0, // 13~16
      0.0,   0.8,    0.0, 0.0  // 17~20
  };
}

void TactileGraspController::pressure_callback(const robotis_interfaces::msg::HandPressures::SharedPtr msg) {
  if (!tactile_sensor_processor_.check_msg(msg)) {
    return;
  }

  const auto sensors = tactile_sensor_processor_.parse_sensors(msg);
  tactile_sensor_processor_.update_pressure(fingers_, baseline_, sensors);
}

void TactileGraspController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  joint_position_map_.clear();

  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    joint_position_map_[msg->name[i]] = msg->position[i];
  }

  joint_state_received_ = true;
}

void TactileGraspController::grasp_state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  const int grasp_state = msg->data;

  if (grasp_state == 3) {
    if (state_ == State::IDLE) {
      reset_for_new_grasp();
      state_ = State::CLOSE;
      RCLCPP_INFO(this->get_logger(), "grasp_state=3 received -> State = CLOSE");
    }
  }
}

void TactileGraspController::control_loop() {
  switch (state_) {
  case State::IDLE:
    handle_idle();
    break;
  case State::CLOSE:
    handle_close();
    break;
  case State::HOLD:
    handle_hold();
    break;
  default:
    break;
  }

  for (int i = 0; i < fingers_num; ++i) {
    prev_filtered_force_[i] = fingers_[i].filtered_force;
  }
}

void TactileGraspController::handle_idle() {
  // IDLE
}

double TactileGraspController::finger_contact_threshold(int finger_idx) const {
  if (finger_idx == 0) {
    return contact_threshold_ * thumb_contact_ratio_;
  }
  return contact_threshold_;
}

void TactileGraspController::handle_close() {
  for (int i = 0; i < fingers_num; ++i) {
    auto& finger = fingers_[i];

    if (!finger.contact_detected) {
      if (i == 0) {
        // thumb : joint3, joint4
        for (int j = 2; j <= 3; ++j) {
          finger.current_joint_targets[j] += close_step_;
          finger.current_joint_targets[j] =
              clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      } else {
        // others : joint2, joint3, joint4
        for (int j = 1; j <= 3; ++j) {
          finger.current_joint_targets[j] += close_step_;
          finger.current_joint_targets[j] =
              clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      }

      if (finger.filtered_force >= finger_contact_threshold(i)) {
        finger.contact_detected = true;
        contact_force_[i] = finger.filtered_force;

        RCLCPP_INFO(this->get_logger(), "[%s] contact detected, force=%.3f", finger.name.c_str(), contact_force_[i]);
      }
    }
  }

  publish_trajectory();

  if (all_fingers_contacted()) {
    set_desired_force_from_contact();
    state_ = State::HOLD;
    RCLCPP_INFO(this->get_logger(), "State -> HOLD");
  }
}

void TactileGraspController::handle_hold() {
  for (int i = 0; i < fingers_num; ++i) {
    auto& finger = fingers_[i];

    double error = desired_force_[i] - finger.filtered_force;
    double dq_scalar = apply_deadband(error);

    if (dq_scalar != 0.0) {
      dq_scalar *= kf_;
    }

    dq_scalar = clamp(dq_scalar, -max_delta_per_step_, max_delta_per_step_);

    if (i == 0) {
      // thumb : joint3, joint4
      const std::array<int, 2> joints = {2, 3};
      const std::array<double, 2> weights = {0.5, 0.5};

      for (int k = 0; k < 2; ++k) {
        const int j = joints[k];
        finger.current_joint_targets[j] += dq_scalar * weights[k];
        finger.current_joint_targets[j] =
            clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
      }
    } else {
      // others : joint2, joint3, joint4
      const std::array<int, 3> joints = {1, 2, 3};
      const std::array<double, 3> weights = {0.5, 0.3, 0.2};

      for (int k = 0; k < 3; ++k) {
        const int j = joints[k];
        finger.current_joint_targets[j] += dq_scalar * weights[k];
        finger.current_joint_targets[j] =
            clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
      }
    }
  }

  publish_trajectory();
}

void TactileGraspController::reset_for_new_grasp() {
  for (int i = 0; i < fingers_num; ++i) {
    fingers_[i].contact_detected = false;
    contact_force_[i] = 0.0;
    desired_force_[i] = 0.0;
    prev_filtered_force_[i] = fingers_[i].filtered_force;
  }

  sync_targets_from_joint_state();
}

void TactileGraspController::set_desired_force_from_contact() {
  for (int i = 0; i < fingers_num; ++i) {
    desired_force_[i] = std::max(contact_force_[i] * force_target_scale_, finger_contact_threshold(i));

    RCLCPP_INFO(this->get_logger(), "[%s] desired_force=%.3f", fingers_[i].name.c_str(), desired_force_[i]);
  }
}

void TactileGraspController::publish_trajectory() {
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = this->now();
  traj_msg.joint_names = all_hand_joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;

  for (const auto& joint_name : all_hand_joint_names_) {
    double position = 0.0;

    if (get_mapped_joint_target(joint_name, position)) {
      point.positions.push_back(position);
    } else {
      point.positions.push_back(get_open_reference_position(joint_name));
    }
  }

  point.time_from_start = rclcpp::Duration::from_seconds(trajectory_dt_);
  traj_msg.points.push_back(point);
  traj_pub_->publish(traj_msg);
}

void TactileGraspController::sync_targets_from_joint_state() {
  if (!joint_state_received_) {
    return;
  }

  for (auto& finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      finger.current_joint_targets[j] = get_joint_position(finger.joint_names[j]);
    }
  }
}

bool TactileGraspController::all_fingers_contacted() const {
  for (const auto& finger : fingers_) {
    if (!finger.contact_detected) {
      return false;
    }
  }
  return true;
}

bool TactileGraspController::get_mapped_joint_target(const std::string& joint_name, double& target) const {
  for (const auto& finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      if (finger.joint_names[j] == joint_name) {
        target = finger.current_joint_targets[j];
        return true;
      }
    }
  }
  return false;
}

double TactileGraspController::get_open_reference_position(const std::string& joint_name) const {
  for (size_t i = 0; i < all_hand_joint_names_.size(); ++i) {
    if (all_hand_joint_names_[i] == joint_name) {
      return open_reference_positions_[i];
    }
  }
  return 0.0;
}

double TactileGraspController::apply_deadband(double error) const {
  if (error > deadband_low_ && error < deadband_high_) {
    return 0.0;
  }
  return error;
}

double TactileGraspController::clamp(double value, double min_v, double max_v) const {
  return std::max(min_v, std::min(value, max_v));
}

double TactileGraspController::get_joint_position(const std::string& joint_name) const {
  auto it = joint_position_map_.find(joint_name);
  if (it != joint_position_map_.end()) {
    return it->second;
  }
  return 0.0;
}

std::string TactileGraspController::state_to_string(State s) const {
  switch (s) {
  case State::IDLE:
    return "IDLE";
  case State::CLOSE:
    return "CLOSE";
  case State::HOLD:
    return "HOLD";
  default:
    return "UNKNOWN";
  }
}

} // namespace robotis_hand_tactile_hold

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotis_hand_tactile_hold::TactileGraspController>());
  rclcpp::shutdown();
  return 0;
}