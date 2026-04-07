#include "tactile_grasp_controller.hpp"

#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace robotis_hand_tactile
{

TactileGraspController::TactileGraspController()
: Node("tactile_grasp_controller")
{
//   declare_parameters();
//   load_parameters();

  init_finger_configs();
  init_joint_name_list();

  tactile_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "tactile_force", 10,
    std::bind(&TactileGraspController::tactile_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&TactileGraspController::joint_state_callback, this, std::placeholders::_1));

  grasp_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/grasp_state", 10,
    std::bind(&TactileGraspController::grasp_state_callback, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "right_hand_controller/joint_trajectory", 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&TactileGraspController::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "TactileGraspController initialized.");
}

void TactileGraspController::init_finger_configs()
{
  // thumb: joint3, joint4
  fingers_[0].name = "thumb";
  fingers_[0].joint_names = {"finger_r_joint3", "finger_r_joint4"};
  fingers_[0].weights = {0.5, 0.5};
  fingers_[0].current_joint_targets = {0.0, 0.0};
  fingers_[0].joint_min = {-1.5, -1.5};
  fingers_[0].joint_max = {1.5, 1.5};

  // index: joint6,7,8
  fingers_[1].name = "index";
  fingers_[1].joint_names = {"finger_r_joint6", "finger_r_joint7", "finger_r_joint8"};
  fingers_[1].weights = {0.5, 0.3, 0.2};
  fingers_[1].current_joint_targets = {0.0, 0.0, 0.0};
  fingers_[1].joint_min = {-1.5, -1.5, -1.5};
  fingers_[1].joint_max = {1.5, 1.5, 1.5};

  // middle: joint10,11,12
  fingers_[2].name = "middle";
  fingers_[2].joint_names = {"finger_r_joint10", "finger_r_joint11", "finger_r_joint12"};
  fingers_[2].weights = {0.5, 0.3, 0.2};
  fingers_[2].current_joint_targets = {0.0, 0.0, 0.0};
  fingers_[2].joint_min = {-1.5, -1.5, -1.5};
  fingers_[2].joint_max = {1.5, 1.5, 1.5};

  // ring: joint14,15,16
  fingers_[3].name = "ring";
  fingers_[3].joint_names = {"finger_r_joint14", "finger_r_joint15", "finger_r_joint16"};
  fingers_[3].weights = {0.5, 0.3, 0.2};
  fingers_[3].current_joint_targets = {0.0, 0.0, 0.0};
  fingers_[3].joint_min = {-1.5, -1.5, -1.5};
  fingers_[3].joint_max = {1.5, 1.5, 1.5};

  // little: joint18,19,20
  fingers_[4].name = "little";
  fingers_[4].joint_names = {"finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};
  fingers_[4].weights = {0.5, 0.3, 0.2};
  fingers_[4].current_joint_targets = {0.0, 0.0, 0.0};
  fingers_[4].joint_min = {-1.5, -1.5, -1.5};
  fingers_[4].joint_max = {1.5, 1.5, 1.5};
}

void TactileGraspController::init_joint_name_list()
{
  all_control_joint_names_.clear();
  for (const auto & finger : fingers_) {
    for (const auto & joint_name : finger.joint_names) {
      all_control_joint_names_.push_back(joint_name);
    }
  }

  all_hand_joint_names_ = {
    "finger_r_joint1",  "finger_r_joint2",  "finger_r_joint3",  "finger_r_joint4",
    "finger_r_joint5",  "finger_r_joint6",  "finger_r_joint7",  "finger_r_joint8",
    "finger_r_joint9",  "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
    "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
    "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
  };

  // open_all 기준 pose
  open_reference_positions_ = {
    0.297,  -1.792, 0.0, 0.0,   // 1~4
    0.0,     1.0,   0.0, 0.0,   // 5~8
    0.0,     1.0,   0.0, 0.0,   // 9~12
    0.0,     1.0,   0.0, 0.0,   // 13~16
    0.0,     1.0,   0.0, 0.0    // 17~20
  };
}

void TactileGraspController::tactile_callback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // Topic 값
  // [thumb_region, thumb_angle, thumb_value,
  //  index_region, index_angle, index_value,
  //  middle_region, middle_angle, middle_value,
  //  ring_region, ring_angle, ring_value,
  //  little_region, little_angle, little_value]

  if (msg->data.size() < 15) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "tactile_force size is smaller than 15");
    return;
  }

  // 
  const std::array<int, 5> value_indices = {2, 5, 8, 11, 14};

  if (use_baseline_ && !baseline_ready_) {
    for (int i = 0; i < k_num_fingers; ++i) {
      fingers_[i].baseline_force += static_cast<double>(msg->data[value_indices[i]]);
    }
    baseline_collected_count_++;

    if (baseline_collected_count_ >= baseline_sample_count_) {
      for (int i = 0; i < k_num_fingers; ++i) {
        fingers_[i].baseline_force /= static_cast<double>(baseline_sample_count_);
      }
      baseline_ready_ = true;
      RCLCPP_INFO(this->get_logger(), "Tactile baseline collected.");
    }
    return;
  }

  for (int i = 0; i < k_num_fingers; ++i) {
    double raw = static_cast<double>(msg->data[value_indices[i]]);

    if (use_baseline_) {
      raw = std::max(0.0, raw - fingers_[i].baseline_force);
    }

    fingers_[i].raw_force = raw;
    fingers_[i].filtered_force =
      alpha_ * raw + (1.0 - alpha_) * fingers_[i].filtered_force;
  }
}

void TactileGraspController::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_position_map_.clear();

  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    joint_position_map_[msg->name[i]] = msg->position[i];
  }

  joint_state_received_ = true;
}

void TactileGraspController::grasp_state_callback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  const int grasp_state = msg->data;

  if (grasp_state == 3) {
    if (state_ == State::IDLE || state_ == State::OPEN) {
      reset_for_new_grasp();
      state_ = State::CLOSE;
      RCLCPP_INFO(this->get_logger(), "grasp_state=3 received -> State = CLOSE");
    }
    return;
  }
}

void TactileGraspController::control_loop()
{
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
    case State::OPEN:
      handle_open();
      break;
    default:
      break;
  }

  for (int i = 0; i < k_num_fingers; ++i) {
    prev_filtered_force_[i] = fingers_[i].filtered_force;
  }
}

void TactileGraspController::handle_idle()
{
     // IDLE
}

void TactileGraspController::handle_close()
{
  for (int i = 0; i < k_num_fingers; ++i) {
    auto & finger = fingers_[i];

    if (!finger.contact_detected) {
      for (size_t j = 0; j < finger.current_joint_targets.size(); ++j) {
        finger.current_joint_targets[j] += close_step_ * finger.weights[j];
        finger.current_joint_targets[j] = clamp(finger.current_joint_targets[j],finger.joint_min[j], finger.joint_max[j]);
      }

      if (finger.filtered_force >= contact_threshold_) {
        finger.contact_detected = true;
        contact_force_[i] = finger.filtered_force;

        RCLCPP_INFO(
          this->get_logger(),
          "[%s] contact detected, force=%.3f",
          finger.name.c_str(), contact_force_[i]);
        std::cout << "contact_threshold_=" << contact_threshold_ << std::endl;
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

void TactileGraspController::handle_hold()
{
  for (int i = 0; i < k_num_fingers; ++i) {
    auto & finger = fingers_[i];

    double error = desired_force_[i] - finger.filtered_force;
    double dq_scalar = apply_deadband(error);

    if (dq_scalar != 0.0) {
      dq_scalar *= kf_;
    }

    dq_scalar = clamp(dq_scalar, -max_delta_per_step_, max_delta_per_step_);

    // if (detect_slip(i)) {
    //   dq_scalar += slip_boost_;
    // }

    for (size_t j = 0; j < finger.current_joint_targets.size(); ++j) {
      finger.current_joint_targets[j] += dq_scalar * finger.weights[j];
      finger.current_joint_targets[j] = clamp(
        finger.current_joint_targets[j],
        finger.joint_min[j], finger.joint_max[j]);
    }
  }

  publish_trajectory();
}

void TactileGraspController::handle_open()
{
  bool all_opened = true;

  for (auto & finger : fingers_) {
    for (size_t j = 0; j < finger.current_joint_targets.size(); ++j) {
      finger.current_joint_targets[j] -= open_step_ * finger.weights[j];
      finger.current_joint_targets[j] = clamp(
        finger.current_joint_targets[j],
        finger.joint_min[j], finger.joint_max[j]);

      if (std::fabs(finger.current_joint_targets[j] - finger.joint_min[j]) > 1e-3) {
        all_opened = false;
      }
    }
  }

  publish_trajectory();

  if (all_opened) {
    state_ = State::IDLE;
    RCLCPP_INFO(this->get_logger(), "All fingers opened. State -> IDLE");
  }
}

void TactileGraspController::reset_for_new_grasp()
{
  for (int i = 0; i < k_num_fingers; ++i) {
    fingers_[i].contact_detected = false;
    contact_force_[i] = 0.0;
    desired_force_[i] = 0.0;
    prev_filtered_force_[i] = fingers_[i].filtered_force;
  }

  sync_targets_from_joint_state();
}

void TactileGraspController::set_desired_force_from_contact()
{
  for (int i = 0; i < k_num_fingers; ++i) {
    desired_force_[i] = std::max(contact_force_[i] * force_target_scale_, contact_threshold_);

    RCLCPP_INFO(
      this->get_logger(),
      "[%s] desired_force=%.3f",
      fingers_[i].name.c_str(), desired_force_[i]);
  }
}

void TactileGraspController::publish_trajectory()
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = this->now();
  traj_msg.joint_names = all_hand_joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;

  for (const auto & joint_name : all_hand_joint_names_) {
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

void TactileGraspController::sync_targets_from_joint_state()
{
  if (!joint_state_received_) {
    return;
  }

  for (auto & finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      finger.current_joint_targets[j] = get_joint_position(finger.joint_names[j]);
    }
  }
}

bool TactileGraspController::all_fingers_contacted() const
{
  for (const auto & finger : fingers_) {
    if (!finger.contact_detected) {
      return false;
    }
  }
  return true;
}

bool TactileGraspController::detect_slip(int finger_idx) const  // 나중에 사용
{
  const double current = fingers_[finger_idx].filtered_force;
  const double prev = prev_filtered_force_[finger_idx];

  return (prev - current) > slip_drop_threshold_;
}

bool TactileGraspController::get_mapped_joint_target(
  const std::string & joint_name, double & target) const
{
  for (const auto & finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      if (finger.joint_names[j] == joint_name) {
        target = finger.current_joint_targets[j];
        return true;
      }
    }
  }
  return false;
}

double TactileGraspController::get_open_reference_position(const std::string & joint_name) const
{
  for (size_t i = 0; i < all_hand_joint_names_.size(); ++i) {
    if (all_hand_joint_names_[i] == joint_name) {
      return open_reference_positions_[i];
    }
  }
  return 0.0;
}

double TactileGraspController::apply_deadband(double error) const
{
  if (error > deadband_low_ && error < deadband_high_) {
    return 0.0;
  }
  return error;
}

double TactileGraspController::clamp(double value, double min_v, double max_v) const
{
  return std::max(min_v, std::min(value, max_v));
}

double TactileGraspController::get_joint_position(const std::string & joint_name) const
{
  auto it = joint_position_map_.find(joint_name);
  if (it != joint_position_map_.end()) {
    return it->second;
  }
  return 0.0;
}

std::string TactileGraspController::state_to_string(State s) const
{
  switch (s) {
    case State::IDLE:
      return "IDLE";
    case State::CLOSE:
      return "CLOSE";
    case State::HOLD:
      return "HOLD";
    case State::OPEN:
      return "OPEN";
    default:
      return "UNKNOWN";
  }
}

}  // namespace robotis_hand_tactile