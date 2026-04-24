#include "tactile_grasp_controller_hold.hpp"
#include "hx5d20_init.hpp"

#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace robotis_hand_tactile_hold {

TactileGraspController::TactileGraspController()
    : Node("tactile_grasp_controller_hold"), tactile_sensor_(this->get_logger(), this->get_clock()) {

  robotis_hand_tactile::declare_params(this);
  param = robotis_hand_tactile::load_params(this);
  tactile_sensor_.set_params(param);

  fingers_ = robotis_hand_tactile::init_fingers();
  hand_joint_names_ = robotis_hand_tactile::init_joint_names();
  init_positions_ = robotis_hand_tactile::init_positions();

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

  unused_finger_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    close_unused_finger();
    publish_traj();
  });

  const auto period = std::chrono::duration<double>(1.0 / param.control_hz);
  control_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                           std::bind(&TactileGraspController::control_loop, this));

  for (auto& finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      finger.current_joint_targets[j] = get_open_pos(finger.joint_names[j]);
    }
  }

  RCLCPP_INFO(this->get_logger(), "TactileGraspController initialized.");
}

void TactileGraspController::pressure_callback(const robotis_interfaces::msg::HandPressures::SharedPtr msg) {
  if (!tactile_sensor_.check_msg(msg)) {
    return;
  }

  const auto sensors = tactile_sensor_.parse_sensors(msg);
  tactile_sensor_.update_pressure(fingers_, baseline_, sensors);
}

void TactileGraspController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  curr_joint_.clear();

  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    curr_joint_[msg->name[i]] = msg->position[i];
  }

  joint_state_received_ = true;
}

void TactileGraspController::grasp_state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  const int grasp_state = msg->data;

  if (grasp_state == 3) {
    if (state_ == State::IDLE) {
      reset_grasp();
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

bool TactileGraspController::unused_finger(int finger_idx) const {
  return std::find(param.un_use_finger.begin(), param.un_use_finger.end(), finger_idx) != param.un_use_finger.end();
}

void TactileGraspController::close_unused_finger() {
  for (int i = 1; i < fingers_num; ++i) {
    if (!unused_finger(i)) {
      continue;
    }
    auto& finger = fingers_[i];

    for (int j = 1; j <= 3; ++j) {
      finger.current_joint_targets[j] += param.close_step * 3;
      finger.current_joint_targets[j] =
          clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
    }
  }
}

void TactileGraspController::handle_idle() {
  // IDLE
}

double TactileGraspController::finger_contact_threshold(int finger_idx) const {
  if (finger_idx == 0) {
    return param.contact_threshold * param.thumb_contact_ratio;
  }
  return param.contact_threshold;
}

void TactileGraspController::handle_close() {
  for (int i = 0; i < fingers_num; ++i) {
    auto& finger = fingers_[i];

    if (unused_finger(i)) {
      finger.contact_detected = true;
      desired_force_[i] = 0.0;
      continue;
    }

    if (!finger.contact_detected) {
      if (i == 0) {
        // const std::array<double, 4> weights = {0.5, 0.5, 0.5, 0.5};
        const std::array<double, 4> weights = {0.5, 0.5, 0.8, 0.2}; // pinch
        // thumb : joint3, joint4
        for (int j = 2; j <= 3; ++j) {
          finger.current_joint_targets[j] += param.close_step * weights[j];
          finger.current_joint_targets[j] =
              clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      } else {
        const std::array<double, 4> weights = {0.0, 0.5, 0.3, 0.2};
        // others : joint2, joint3, joint4
        for (int j = 1; j <= 3; ++j) {
          finger.current_joint_targets[j] += param.close_step * weights[j];
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

  publish_traj();

  if (all_contacted()) {
    set_desired_force();
    // state_ = State::HOLD;
    // RCLCPP_INFO(this->get_logger(), "State -> HOLD");
    state_ = State::IDLE;
    RCLCPP_INFO(this->get_logger(), "State -> IDLE"); // pinch
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

    dq_scalar = clamp(dq_scalar, -reactive_step_, reactive_step_);

    if (i == 0) {
      // thumb : joint3, joint4
      const std::array<int, 2> joints = {2, 3};
      const std::array<double, 2> weights = {0.7, 0.3};

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

  publish_traj();
}

void TactileGraspController::reset_grasp() {
  for (int i = 0; i < fingers_num; ++i) {
    fingers_[i].contact_detected = false;
    contact_force_[i] = 0.0;
    desired_force_[i] = 0.0;
    prev_filtered_force_[i] = fingers_[i].filtered_force;
  }

  sync_targets();
}

void TactileGraspController::set_desired_force() {
  for (int i = 0; i < fingers_num; ++i) {
    desired_force_[i] = std::max(contact_force_[i] * param.reactive_force, finger_contact_threshold(i));

    RCLCPP_INFO(this->get_logger(), "[%s] desired_force=%.3f", fingers_[i].name.c_str(), desired_force_[i]);
  }
}

void TactileGraspController::publish_traj() {
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = this->now();
  traj_msg.joint_names = hand_joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;

  for (const auto& joint_name : hand_joint_names_) {
    double position = 0.0;

    if (get_target(joint_name, position)) {
      point.positions.push_back(position);
    } else {
      point.positions.push_back(get_open_pos(joint_name));
    }
  }

  point.time_from_start = rclcpp::Duration::from_seconds(param.trajectory_dt);
  traj_msg.points.push_back(point);
  traj_pub_->publish(traj_msg);
}

void TactileGraspController::sync_targets() {
  if (!joint_state_received_) {
    return;
  }

  for (auto& finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      finger.current_joint_targets[j] = get_joint_pos(finger.joint_names[j]);
    }
  }
}

bool TactileGraspController::all_contacted() const {
  for (const auto& finger : fingers_) {
    if (!finger.contact_detected) {
      return false;
    }
  }
  return true;
}

bool TactileGraspController::get_target(const std::string& joint_name, double& target) const {
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

double TactileGraspController::get_open_pos(const std::string& joint_name) const {
  for (size_t i = 0; i < hand_joint_names_.size(); ++i) {
    if (hand_joint_names_[i] == joint_name) {
      return init_positions_[i];
    }
  }
  return 0.0;
}

double TactileGraspController::apply_deadband(double error) const {
  if (error > deadband_L && error < deadband_H) {
    return 0.0;
  }
  return error;
}

double TactileGraspController::clamp(double value, double min_v, double max_v) const {
  return std::max(min_v, std::min(value, max_v));
}

double TactileGraspController::get_joint_pos(const std::string& joint_name) const {
  auto it = curr_joint_.find(joint_name);
  if (it != curr_joint_.end()) {
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