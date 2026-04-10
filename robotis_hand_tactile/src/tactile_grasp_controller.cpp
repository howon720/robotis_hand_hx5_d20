#include "tactile_grasp_controller.hpp"

using namespace std::chrono_literals;

namespace robotis_hand_tactile {

TactileGraspController::TactileGraspController()
: Node("tactile_grasp_controller")
{
  init_fingers();
  init_joints();
  init_tactiles();

  std::array<FingerPlanarIk::FingerModel, 4> ik_models{};

  for (int i = 0; i < 4; ++i) {
    const int finger_idx = i + 1;  // index=1, middle=2, ring=3, little=4

    ik_models[i].joint_min = {
      fingers_[finger_idx].joint_min[1],
      fingers_[finger_idx].joint_min[2],
      fingers_[finger_idx].joint_min[3]
    };

    ik_models[i].joint_max = {
      fingers_[finger_idx].joint_max[1],
      fingers_[finger_idx].joint_max[2],
      fingers_[finger_idx].joint_max[3]
    };

    // link
    ik_models[i].link_lengths = {0.0235, 0.0355, 0.0355};
  }

  finger_planar_ik_ = std::make_unique<FingerPlanarIk>(ik_models);

  pressure_sub_ = this->create_subscription<robotis_interfaces::msg::HandPressures>(
    "/right_hand/finger_pressures", 10,
    std::bind(&TactileGraspController::on_pressure, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&TactileGraspController::on_joint_state, this, std::placeholders::_1));

  grasp_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/grasp_state", 10,
    std::bind(&TactileGraspController::on_grasp_state, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/right_hand_controller/joint_trajectory", 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&TactileGraspController::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "TactileGraspController initialized.");
}

void TactileGraspController::init_fingers()
{
  // thumb
  fingers_[0].name = "thumb";
  fingers_[0].joint_names = {"finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4"};
  fingers_[0].joint_min = {-1.5, -3.5, -1.5, -1.5};
  fingers_[0].joint_max = { 1.5,  0.5,  1.5,  1.5};

  // index
  fingers_[1].name = "index";
  fingers_[1].joint_names = {"finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8"};
  fingers_[1].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[1].joint_max = { 0.6,  1.5,  1.5,  1.5};

  // middle      cylinder_tape : 0.4
  fingers_[2].name = "middle";
  fingers_[2].joint_names = {"finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12"};
  fingers_[2].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[2].joint_max = { 0.6,  1.5,  1.5,  1.5};

  // ring        cylinder_tape : 0.4
  fingers_[3].name = "ring";
  fingers_[3].joint_names = {"finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16"};
  fingers_[3].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[3].joint_max = { 0.6,  1.5,  1.5,  1.5};

  // little
  fingers_[4].name = "little";
  fingers_[4].joint_names = {"finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};
  fingers_[4].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[4].joint_max = { 0.6,  1.5,  1.5,  1.5};

  for (auto & finger : fingers_) {
    finger.current_joint_targets = {0.0, 0.0, 0.0, 0.0};
  }
}

void TactileGraspController::init_joints()
{
  all_hand_joint_names_ = {
    "finger_r_joint1",  "finger_r_joint2",  "finger_r_joint3",  "finger_r_joint4",
    "finger_r_joint5",  "finger_r_joint6",  "finger_r_joint7",  "finger_r_joint8",
    "finger_r_joint9",  "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
    "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
    "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
  };

  open_reference_positions_ = {    // 1.0
    0.297, -1.792, 0.0, 0.0,     // org
    // 0.12, -1.68, 0.0, 0.0,     // tennis
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0
  };
}

void TactileGraspController::init_tactiles()
{
  const double x_offset = tactile_x_ / 3.0;
  const double y_offset = tactile_y_ / 3.0;

  const std::array<double, 3> xs = {-x_offset, 0.0, x_offset};
  const std::array<double, 3> ys = {-y_offset, 0.0, y_offset};

  int idx = 0;
  for (double y : ys) {
    for (double x : xs) {
      tactile_xy_[idx++] = {x, y};
    }
  }
}

bool TactileGraspController::check_msg(
  const robotis_interfaces::msg::HandPressures::SharedPtr msg) const
{
  if (msg->sensors.size() != fingers_num) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "sensors size mismatch: %zu", msg->sensors.size());
    return false;
  }

  for (size_t i = 0; i < msg->sensors.size(); ++i) {
    if (msg->sensors[i].pressure_names.size() != tactiles_num) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "sensor[%zu] pressure_names size mismatch: %zu",
                          i, msg->sensors[i].pressure_names.size());
      return false;
    }

    if (msg->sensors[i].pressure_values.size() != tactiles_num) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "sensor[%zu] pressure_values size mismatch: %zu",
                          i, msg->sensors[i].pressure_values.size());
      return false;
    }
  }

  return true;
}

std::array<Hx5d20SensorData, TactileGraspController::fingers_num>
TactileGraspController::parse_sensors(
  const robotis_interfaces::msg::HandPressures::SharedPtr msg) const
{
  std::array<Hx5d20SensorData, fingers_num> out{};

  for (size_t i = 0; i < fingers_num; ++i) {
    out[i].name = msg->sensors[i].sensor_name;

    for (size_t j = 0; j < tactiles_num; ++j) {
      out[i].labels[j] = msg->sensors[i].pressure_names[j];
      out[i].values[j] = static_cast<double>(msg->sensors[i].pressure_values[j]);
    }
  }

  return out;
}

void TactileGraspController::on_pressure(
  const robotis_interfaces::msg::HandPressures::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!check_msg(msg)) {
    return;
  }

  const auto sensors = parse_sensors(msg);
  update_pressure(sensors);
}

void TactileGraspController::update_pressure(
  const std::array<Hx5d20SensorData, fingers_num> & sensors)
{
  if (!baseline_) {
    for (int f = 0; f < fingers_num; ++f) {
      for (int t = 0; t < tactiles_num; ++t) {
        fingers_[f].baseline_sum_tactiles[t] += sensors[f].values[t];
      }
      fingers_[f].baseline_samples++;
    }

    bool ready = true;
    for (int f = 0; f < fingers_num; ++f) {
      if (fingers_[f].baseline_samples < baseline_sample_count_) {
        ready = false;
        break;
      }
    }

    if (ready) {
      for (int f = 0; f < fingers_num; ++f) {
        const int count = std::max(1, fingers_[f].baseline_samples);
        for (int t = 0; t < tactiles_num; ++t) {
          fingers_[f].baseline_tactiles[t] =
            fingers_[f].baseline_sum_tactiles[t] / static_cast<double>(count);
          fingers_[f].ema_tactiles[t] = 0.0;
        }
      }
      baseline_ = true;
      RCLCPP_INFO(this->get_logger(), "Tactile-wise baseline ready.");
    }
    return;
  }

  for (int f = 0; f < fingers_num; ++f) {
    std::array<double, tactiles_num> filtered{};
    double total_force = 0.0;

    for (int t = 0; t < tactiles_num; ++t) {
      double v = sensors[f].values[t];

      v -= fingers_[f].baseline_tactiles[t];
      
      if (v < 0.0) {  // baseline 보다 압력이 낮아질 시 0으로 고정
        v = 0.0;
      }

      fingers_[f].ema_tactiles[t] =
        (1.0 - ema_alpha_) * fingers_[f].ema_tactiles[t] + ema_alpha_ * v;

      filtered[t] = fingers_[f].ema_tactiles[t];
      total_force += filtered[t];
    }

    fingers_[f].filtered_force =
      (1.0 - ema_alpha_) * fingers_[f].filtered_force + ema_alpha_ * total_force;

    fingers_[f].cop = calc_cop(f, filtered);
  }
}

TactileGraspController::CopInfo TactileGraspController::calc_cop(int finger_idx, 
  const std::array<double, tactiles_num> & p) const
{
  CopInfo info;
  info.pressure = p;
  info.total_force = std::accumulate(p.begin(), p.end(), 0.0);

  if (info.total_force <= 1e-9) {
    return info;
  }

  for (int i = 0; i < tactiles_num; ++i) {
    info.cop_x += p[i] * tactile_xy_[i].first;
    info.cop_y += p[i] * tactile_xy_[i].second;
  }
  info.cop_x /= info.total_force;
  info.cop_y /= info.total_force;

  info.top_sum = p[0] + p[1] + p[2];
  info.mid_sum = p[3] + p[4] + p[5];
  info.bot_sum = p[6] + p[7] + p[8];

  info.left_sum   = p[0] + p[3] + p[6];
  info.center_sum = p[1] + p[4] + p[7];
  info.right_sum  = p[2] + p[5] + p[8];

  info.top_x_bias = (-1.0 * p[0]) + (0.0 * p[1]) + (1.0 * p[2]);
  info.mid_x_bias = (-1.0 * p[3]) + (0.0 * p[4]) + (1.0 * p[5]);
  info.bot_x_bias = (-1.0 * p[6]) + (0.0 * p[7]) + (1.0 * p[8]);

  // normalize CoP to [-1, 1] using half size of tactile sensor
  const double half_x = tactile_x_ * 0.5;
  const double half_y = tactile_y_ * 0.5;

  info.cop_x_ratio = clamp(info.cop_x / std::max(half_x, 1e-9), -1.0, 1.0);
  info.cop_y_ratio = clamp(info.cop_y / std::max(half_y, 1e-9), -1.0, 1.0);

  // Y_LEFT / Y_RIGHT decision is based on left-right CoP position
  // center area is wider and configurable by y_center_ratio_threshold_
  const double abs_x_ratio = std::fabs(info.cop_x_ratio);

  double y__ = y_center_ratio_threshold_ + ((finger_idx == 0) ? 0.45 : 0.0);
  double x__ = x_center_ratio_threshold_ + ((finger_idx == 0) ? 0.45 : 0.0);

  if (abs_x_ratio > y__) {
    const double lateral_cost =
      (abs_x_ratio - y__) /
      std::max(1.0 - y__, 1e-9);

    const double clipped_cost = clamp(lateral_cost, 0.0, 1.0);

    if (info.cop_x_ratio < 0.0) {
      info.y_left_cost = clipped_cost;
      info.y_right_cost = 0.0;
    } else {
      info.y_left_cost = 0.0;
      info.y_right_cost = clipped_cost;
    }
  }

  // X_TOP / X_BOT decision is based on top-bottom CoP position
  const double abs_y_ratio = std::fabs(info.cop_y_ratio);

  if (abs_y_ratio > x__) {
    const double vertical_cost =
      (abs_y_ratio - x__) /
      std::max(1.0 - x__, 1e-9);

    const double clipped_cost = clamp(vertical_cost, 0.0, 1.0);

    if (info.cop_y_ratio < 0.0) {
      info.x_top_cost = clipped_cost;
      info.x_bot_cost = 0.0;
    } else {
      info.x_top_cost = 0.0;
      info.x_bot_cost = clipped_cost;
    }
  }

  return info;
}

std::optional<TactileGraspController::CorrectionDecision>
TactileGraspController::pick_correction(const CopInfo & info) const
{
  if (info.total_force < min_force_for_correction_) {
    return std::nullopt;
  }

  // X_TOP / X_BOT by CoP ratio-based cost
  const double vertical_cost = std::max(info.x_top_cost, info.x_bot_cost);
  if (vertical_cost >= x_cost_trigger_threshold_) {
    if (info.x_top_cost > info.x_bot_cost) {
      return CorrectionDecision{CorrectionType::X_TOP, info.x_top_cost};
    } else {
      return CorrectionDecision{CorrectionType::X_BOT, info.x_bot_cost};
    }
  }

  // Y_LEFT / Y_RIGHT by CoP ratio-based cost
  const double lateral_cost = std::max(info.y_left_cost, info.y_right_cost);
  if (lateral_cost >= y_cost_trigger_threshold_) {
    if (info.y_left_cost > info.y_right_cost) {
      return CorrectionDecision{CorrectionType::Y_LEFT, info.y_left_cost};
    } else {
      return CorrectionDecision{CorrectionType::Y_RIGHT, info.y_right_cost};
    }
  }

  return std::nullopt;
}

void TactileGraspController::on_joint_state(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  joint_position_map_.clear();

  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    joint_position_map_[msg->name[i]] = msg->position[i];
  }

  joint_received_ = true;
}

void TactileGraspController::on_grasp_state(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->data == 3) {
    if (state_ == State::IDLE || state_ == State::OPEN) {
      reset_grasp();
      state_ = State::CLOSE;
      RCLCPP_INFO(this->get_logger(), "grasp_state=3 -> CLOSE");
    }
  }
}

// main loop
void TactileGraspController::control_loop()
{
  std::lock_guard<std::mutex> lock(mutex_);

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
}

void TactileGraspController::handle_idle()
{
  // IDLE
}

double TactileGraspController::finger_contact_threshold(int finger_idx) const
{
  if (finger_idx == 0) {
    return contact_threshold_ * thumb_contact_ratio_;
  }
  return contact_threshold_;
}

void TactileGraspController::handle_close()
{
  for (int i = 0; i < fingers_num; ++i) {
    auto & finger = fingers_[i];

    if (!finger.contact_detected) {
      // thumb 
      if (i == 0) {
        for (int j = 2; j <= 3; ++j) {
          finger.current_joint_targets[j] += close_step_;
          finger.current_joint_targets[j] = clamp(
            finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      }
      // other 
      else {
        for (int j = 1; j <= 3; ++j) {
          finger.current_joint_targets[j] += close_step_;
          finger.current_joint_targets[j] = clamp(
            finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      }

      if (finger.filtered_force >= finger_contact_threshold(i)) {
        finger.contact_detected = true;
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] contact detected, force=%.2f",
          finger.name.c_str(), finger.filtered_force);
      }
    }
  }

  publish_traj();

  if (all_contacted()) {
    set_desired_force();
    state_ = State::HOLD;
    RCLCPP_INFO(this->get_logger(), "State -> HOLD");
  }
}

void TactileGraspController::set_desired_force()
{
  for (int i = 0; i < fingers_num; ++i) {
    desired_force_[i] = std::max(fingers_[i].filtered_force, finger_contact_threshold(i));
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] desired_force set to %.2f",
      fingers_[i].name.c_str(),
      desired_force_[i]);
  }
}

void TactileGraspController::regulate_grasp_force(int finger_idx)
{
  auto & finger = fingers_[finger_idx];
  const double error = desired_force_[finger_idx] - finger.filtered_force;

  if (error > -deadband_L && error < deadband_H) {
    return;
  }

  double dq = force_kp_ * error;
  dq = clamp(dq, -feedback_max_delta_, feedback_max_delta_);

  const double scale = finger_step_scale(finger_idx);
  dq *= scale;

  // check + -
  if (dq > 0.0) {
    grasp_step_hold(finger_idx, dq);
  } else if (dq < 0.0) {
    release_step_hold(finger_idx, -dq);
  }
}

void TactileGraspController::grasp_step_hold(int finger_idx, double step)
{
  auto & finger = fingers_[finger_idx];

  if (finger_idx == 0) {
    const std::array<int, 2> joints = {2, 3};
    const std::array<double, 2> ratios = {6.0, 4.0};
    const double sum = ratios[0] + ratios[1];

    for (int k = 0; k < 2; ++k) {
      const int j = joints[k];
      finger.current_joint_targets[j] += step * (ratios[k] / sum);
      finger.current_joint_targets[j] = clamp(
        finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
    }
    return;
  }

  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, +step);
}

void TactileGraspController::release_step_hold(int finger_idx, double step)
{
  auto & finger = fingers_[finger_idx];

  if (finger_idx == 0) {
    const std::array<int, 2> joints = {2, 3};
    const std::array<double, 2> ratios = {6.0, 4.0};
    const double sum = ratios[0] + ratios[1];

    for (int k = 0; k < 2; ++k) {
      const int j = joints[k];
      finger.current_joint_targets[j] -= step * (ratios[k] / sum);
      finger.current_joint_targets[j] = clamp(
        finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
    }
    return;
  }

  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, -step);
}

void TactileGraspController::update_little_joint1()
{
  auto & little = fingers_[4];
  const double curr = little.current_joint_targets[0];
  const double delta = std::fabs(curr - little.prev_joint1_target);

  little.joint1_motion_accum += delta;
  little.prev_joint1_target = curr;
}

bool TactileGraspController::TL_regrasp_flag() const
{
  if (TL_regrasp_mode_) {
    return false;
  }

  return fingers_[4].joint1_motion_accum >= little_regrasp_delta_;
}

bool TactileGraspController::TL_regrasp_done() const
{
  const double thumb_target = finger_contact_threshold(0);
  const double little_target = finger_contact_threshold(4) * regrasp_force_ratio_;

  return fingers_[0].filtered_force >= thumb_target &&
         fingers_[4].filtered_force >= little_target;
}

void TactileGraspController::reset_TL_regrasp()
{
  TL_regrasp_mode_ = false;
  fingers_[4].joint1_motion_accum = 0.0;
  fingers_[4].prev_joint1_target = fingers_[4].current_joint_targets[0];

  for (auto & plan : correction_plans_) {
    plan = CorrectionPlan{};
  }
}

// old
void TactileGraspController::handle_hold()
{
  update_little_joint1();

  if (TL_regrasp_flag()) {
    TL_regrasp_mode_ = true;

    for (int i = 0; i < fingers_num; ++i) {
      correction_plans_[i] = CorrectionPlan{};
    }

    RCLCPP_INFO(
      this->get_logger(),
      "thumb+little regrasp mode start (little joint1 accum=%.3f)",
      fingers_[4].joint1_motion_accum);
  }

  if (TL_regrasp_mode_) {

    grasp_step_hold(0, TL_grasp_step_);
    grasp_step_hold(4, TL_grasp_step_);

    if (TL_regrasp_done()) {
      reset_TL_regrasp();
      RCLCPP_INFO(this->get_logger(), "thumb+little regrasp mode done -> resume CoP");
    }

    publish_traj();
    return;
  }

  for (int i = 0; i < fingers_num; ++i) {
    if (correction_plans_[i].active) {
      apply_correction(i);
      continue;
    }

    regulate_grasp_force(i);
    start_correction(i);
  }

  publish_traj();

  if (all_corrections_blocked()) {
    state_ = State::IDLE;
    RCLCPP_INFO(this->get_logger(), "All available corrections blocked -> IDLE");
  }
}

// void TactileGraspController::handle_hold()
// {
//   for (int i = 0; i < fingers_num; ++i) {
//     if (need_regrasp(i)) {
//       correction_plans_[i] = CorrectionPlan{};
//     }
//     if (update_regrasp(i)) {
//       continue;
//     }

//     if (correction_plans_[i].active) {
//       apply_correction(i);
//       continue;
//     }

//     regulate_grasp_force(i);

//     if (!need_regrasp(i)) {
//       start_correction(i);
//     }
//   }

//   publish_traj();
// }

void TactileGraspController::handle_open()
{
  bool all_opened = true;

  for (auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      finger.current_joint_targets[j] -= open_step_;
      finger.current_joint_targets[j] = clamp(
        finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);

      if (std::fabs(finger.current_joint_targets[j] - finger.joint_min[j]) > 1e-3) {
        all_opened = false;
      }
    }
  }

  publish_traj();

  if (all_opened) {
    state_ = State::IDLE;
    RCLCPP_INFO(this->get_logger(), "All fingers opened -> IDLE");
  }
}

void TactileGraspController::start_correction(int finger_idx)
{
  // // thumb X
  // if (finger_idx == 0) {
  //   return;
  // }

  if (TL_regrasp_mode_) {
    return;
  }

  const auto maybe_decision = pick_correction(fingers_[finger_idx].cop);
  if (!maybe_decision.has_value()) {
    return;
  }

  // max_min limit
  if (!can_run_correction_step(finger_idx, maybe_decision->type)) {
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] correction skipped: %s (joint limit)",
      fingers_[finger_idx].name.c_str(),
      correction_str(maybe_decision->type).c_str());
    return;
  }

  auto & plan = correction_plans_[finger_idx];
  plan.type = maybe_decision->type;
  plan.cost = maybe_decision->cost;
  plan.phase = 0;
  plan.active = true;

  switch (plan.type) {
    case CorrectionType::X_TOP:
    case CorrectionType::X_BOT:
    case CorrectionType::Y_LEFT:
    case CorrectionType::Y_RIGHT:
      plan.ticks_remaining = phase_step_;
      break;
    default:
      plan.active = false;
      break;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] correction start: %s (cost=%.3f, cop_x_ratio=%.3f, cop_y_ratio=%.3f)",
    fingers_[finger_idx].name.c_str(),
    correction_str(plan.type).c_str(),
    plan.cost,
    fingers_[finger_idx].cop.cop_x_ratio,
    fingers_[finger_idx].cop.cop_y_ratio);
}

void TactileGraspController::apply_correction(int finger_idx)
{
  auto & plan = correction_plans_[finger_idx];
  const bool done = run_correction(finger_idx, plan);

  if (done) {
    plan = CorrectionPlan{};
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] correction done",
      fingers_[finger_idx].name.c_str());
  }
}

bool TactileGraspController::run_correction(int finger_idx, CorrectionPlan & plan)
{
  if (!plan.active) {
    return true;
  }

  const double scale = finger_step_scale(finger_idx);

  switch (plan.type) {
    // case CorrectionType::X_TOP:
    // {
    //   if (plan.phase == 0) {
    //     release_234(finger_idx, release_step_base_ * scale);
    //     plan.ticks_remaining--;
    //     if (plan.ticks_remaining <= 0) {
    //       plan.phase = 1;
    //       plan.ticks_remaining = phase_step_;
    //     }
    //     return false;
    //   }

    //   if (plan.phase == 1) {
    //     apply_ratio_step(finger_idx, {1, 2, 3}, {2.0, 3.0, 5.0}, + grasp_step_base_ * scale);
    //     plan.ticks_remaining--;
    //     return (plan.ticks_remaining <= 0);
    //   }
    //   return true;
    // }

    // case CorrectionType::X_BOT:
    // {
    //   if (plan.phase == 0) {
    //     mixed_bot_step(finger_idx, release_step_base_ * scale);
    //     plan.ticks_remaining--;
    //     if (plan.ticks_remaining <= 0) {
    //       plan.phase = 1;
    //       plan.ticks_remaining = phase_step_;
    //     }
    //     return false;
    //   }

    //   if (plan.phase == 1) {
    //     apply_ratio_step(finger_idx, {1, 2, 3}, {2.0, 3.0, 5.0}, +grasp_step_base_ * scale);
    //     plan.ticks_remaining--;
    //     return (plan.ticks_remaining <= 0);
    //   }
    //   return true;
    // }

    case CorrectionType::X_TOP:
    {
      const bool moved = apply_x_correction_by_ik(finger_idx, true, plan.cost);
      plan.ticks_remaining--;
      return (!moved || plan.ticks_remaining <= 0);
    }

    case CorrectionType::X_BOT:
    {
      const bool moved = apply_x_correction_by_ik(finger_idx, false, plan.cost);
      plan.ticks_remaining--;
      return (!moved || plan.ticks_remaining <= 0);
    }


    case CorrectionType::Y_LEFT:
    {
      if (plan.phase == 0) {
        release_234(finger_idx, release_step_base_ * scale);
        plan.ticks_remaining--;
        if (plan.ticks_remaining <= 0) {
          plan.phase = 1;
          plan.ticks_remaining = phase_step_;
        }
        return false;
      }

      if (plan.phase == 1) {
        if (finger_idx == 0) {
          shift_thumb_y(finger_idx, joint1_shift_step_, joint1_shift_step_);
        } else {
          shift_joint1(finger_idx, -joint1_shift_step_);
        }

        plan.ticks_remaining--;
        if (plan.ticks_remaining <= 0) {
          plan.phase = 2;
          plan.ticks_remaining = phase_step_;
        }
        return false;
      }

      if (plan.phase == 2) {
        grasp_234(finger_idx, grasp_step_base_ * scale);
        plan.ticks_remaining--;
        return (plan.ticks_remaining <= 0);
      }
      return true;
    }

    case CorrectionType::Y_RIGHT:
    {
      if (plan.phase == 0) {
        release_234(finger_idx, release_step_base_ * scale);
        plan.ticks_remaining--;
        if (plan.ticks_remaining <= 0) {
          plan.phase = 1;
          plan.ticks_remaining = phase_step_;
        }
        return false;
      }

      if (plan.phase == 1) {
        if (finger_idx == 0) {
          // thumb: joint1, joint2 둘 다 -- 방향
          shift_thumb_y(finger_idx, -joint1_shift_step_, -joint1_shift_step_);
        } else {
          shift_joint1(finger_idx, +joint1_shift_step_);
        }

        plan.ticks_remaining--;
        if (plan.ticks_remaining <= 0) {
          plan.phase = 2;
          plan.ticks_remaining = phase_step_;
        }
        return false;
      }

      if (plan.phase == 2) {
        grasp_234(finger_idx, grasp_step_base_ * scale);
        plan.ticks_remaining--;
        return (plan.ticks_remaining <= 0);
      }
      return true;
    }

    default:
      return true;
  }
}

void TactileGraspController::apply_ratio_step(
  int finger_idx,
  const std::array<int, 3> & local_joint_ids,
  const std::array<double, 3> & ratios,
  double signed_step)
{
  auto & finger = fingers_[finger_idx];
  const double ratio_sum = ratios[0] + ratios[1] + ratios[2];

  for (int i = 0; i < 3; ++i) {
    const int j = local_joint_ids[i];
    const double delta = signed_step * (ratios[i] / ratio_sum);

    finger.current_joint_targets[j] += delta;
    finger.current_joint_targets[j] = clamp(
      finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
  }
}

bool TactileGraspController::need_regrasp(int finger_idx) const
{
  const auto & finger = fingers_[finger_idx];
  const double trigger_force = desired_force_[finger_idx] * regrasp_trigger_ratio_;
  return finger.filtered_force < trigger_force;
}

bool TactileGraspController::update_regrasp(int finger_idx)
{
  auto & finger = fingers_[finger_idx];
  // force low 시 regrasp
  if (finger.filtered_force < desired_force_[finger_idx] * regrasp_trigger_ratio_) {
    finger.regrasp_mode = true;
    finger.regrasp_stable_ticks = 0;
  }

  if (!finger.regrasp_mode) {
    return false;
  }

  grasp_step_hold(finger_idx, regrasp_step_);

  if (finger.filtered_force >= desired_force_[finger_idx]) {
    finger.regrasp_stable_ticks++;
  } else {
    finger.regrasp_stable_ticks = 0;
  }

  if (finger.regrasp_stable_ticks >= regrasp_stable_count_) {
    finger.regrasp_mode = false;
    finger.regrasp_stable_ticks = 0;
    return false;  // 이제 normal hold로 돌아감
  }

  return true;  // 아직 재그립 중
}

double TactileGraspController::max_filtered_force() const
{
  double max_force = 0.0;
  for (int i = 1; i < fingers_num; ++i) {   // thumb 제외
    max_force = std::max(max_force, fingers_[i].filtered_force);
  }
  return max_force;
}

double TactileGraspController::finger_step_scale(int finger_idx) const
{
  if (finger_idx == 0) {
    return min_step_scale_;
  }

  const double f_max = max_filtered_force();

  if (f_max < 1e-6) {
    return 1.0;
  }

  const double ratio = fingers_[finger_idx].filtered_force / f_max;
  return clamp(ratio, min_step_scale_, max_step_scale_);
}

void TactileGraspController::shift_joint1(int finger_idx, double delta)
{
  auto & finger = fingers_[finger_idx];

  // const int joint_idx = (finger_idx == 0) ? 1 : 0;
  const int joint_idx = 0;

  finger.current_joint_targets[joint_idx] += delta;
  finger.current_joint_targets[joint_idx] = clamp(
    finger.current_joint_targets[joint_idx],
    finger.joint_min[joint_idx],
    finger.joint_max[joint_idx]);
}

void TactileGraspController::shift_thumb_y(int finger_idx, double delta1, double delta2)
{
  auto & finger = fingers_[finger_idx];

  finger.current_joint_targets[0] += delta1 * 0.1;
  finger.current_joint_targets[1] += delta2 * 0.1;

  finger.current_joint_targets[0] = clamp(
    finger.current_joint_targets[0],
    finger.joint_min[0],
    finger.joint_max[0]);

  finger.current_joint_targets[1] = clamp(
    finger.current_joint_targets[1],
    finger.joint_min[1],
    finger.joint_max[1]);
}

void TactileGraspController::grasp_234(int finger_idx, double step)
{
  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, +step);
}

void TactileGraspController::release_234(int finger_idx, double step)
{
  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, -step);
}

void TactileGraspController::mixed_bot_step(int finger_idx, double step)
{
  auto & finger = fingers_[finger_idx];
  const double sum = 5.0 + 3.0 + 2.0;

  finger.current_joint_targets[1] -= step * (5.0 / sum);
  finger.current_joint_targets[2] -= step * (3.0 / sum);
  finger.current_joint_targets[3] += step * (2.0 / sum);

  for (int j = 1; j <= 3; ++j) {
    finger.current_joint_targets[j] = clamp(
      finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
  }
}

void TactileGraspController::reset_grasp()
{
  for (auto & finger : fingers_) {
    finger.contact_detected = false;
    finger.regrasp_mode = false;
    finger.regrasp_stable_ticks = 0;
    finger.joint1_motion_accum = 0.0;
    finger.prev_joint1_target = finger.current_joint_targets[0];
  }

  for (auto & plan : correction_plans_) {
    plan = CorrectionPlan{};
  }

  TL_regrasp_mode_ = false;

  sync_targets();

  for (auto & finger : fingers_) {
    finger.prev_joint1_target = finger.current_joint_targets[0];
  }
}

void TactileGraspController::sync_targets()
{
  if (!joint_received_) {
    return;
  }

  for (auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      finger.current_joint_targets[j] = get_joint_pos(finger.joint_names[j]);
    }
  }
}

bool TactileGraspController::all_contacted() const
{
  for (const auto & finger : fingers_) {
    if (!finger.contact_detected) {
      return false;
    }
  }
  return true;
}

bool TactileGraspController::can_run_correction_step(int finger_idx, CorrectionType type) const
{
  const auto & finger = fingers_[finger_idx];
  const double eps = 1e-6;

  switch (type) {
    case CorrectionType::Y_LEFT:
      if (finger_idx == 0) {
        return (finger.current_joint_targets[0] < finger.joint_max[0] - eps) ||
               (finger.current_joint_targets[1] < finger.joint_max[1] - eps);
      } else {
        return finger.current_joint_targets[0] > finger.joint_min[0] + eps;
      }

    case CorrectionType::Y_RIGHT:
      if (finger_idx == 0) {
        return (finger.current_joint_targets[0] > finger.joint_min[0] + eps) ||
               (finger.current_joint_targets[1] > finger.joint_min[1] + eps);
      } else {
        return finger.current_joint_targets[0] < finger.joint_max[0] - eps;
      }

    // case CorrectionType::X_TOP:
    //   return (finger.current_joint_targets[1] > finger.joint_min[1] + eps) ||
    //          (finger.current_joint_targets[2] > finger.joint_min[2] + eps) ||
    //          (finger.current_joint_targets[3] > finger.joint_min[3] + eps) ||
    //          (finger.current_joint_targets[1] < finger.joint_max[1] - eps) ||
    //          (finger.current_joint_targets[2] < finger.joint_max[2] - eps) ||
    //          (finger.current_joint_targets[3] < finger.joint_max[3] - eps);

    // case CorrectionType::X_BOT:
    //   return (finger.current_joint_targets[1] > finger.joint_min[1] + eps) ||
    //          (finger.current_joint_targets[2] > finger.joint_min[2] + eps) ||
    //          (finger.current_joint_targets[3] < finger.joint_max[3] - eps) ||
    //          (finger.current_joint_targets[1] < finger.joint_max[1] - eps) ||
    //          (finger.current_joint_targets[2] < finger.joint_max[2] - eps) ||
    //          (finger.current_joint_targets[3] < finger.joint_max[3] - eps);

    case CorrectionType::X_TOP:  // IK
    case CorrectionType::X_BOT:
      return (finger.current_joint_targets[1] > finger.joint_min[1] + eps &&
              finger.current_joint_targets[1] < finger.joint_max[1] - eps) ||
            (finger.current_joint_targets[2] > finger.joint_min[2] + eps &&
              finger.current_joint_targets[2] < finger.joint_max[2] - eps) ||
            (finger.current_joint_targets[3] > finger.joint_min[3] + eps &&
              finger.current_joint_targets[3] < finger.joint_max[3] - eps);

    default:
      return false;
  }
}

bool TactileGraspController::all_corrections_blocked() const
{
  for (int i = 0; i < fingers_num; ++i) {
    const auto maybe_decision = pick_correction(fingers_[i].cop);
    if (!maybe_decision.has_value()) {
      continue;
    }

    if (can_run_correction_step(i, maybe_decision->type)) {
      return false;
    }
  }
  return true;
}

std::array<double, 3> TactileGraspController::get_planar_q(int finger_idx) const
{
  const auto & finger = fingers_[finger_idx];
  return {
    get_joint_pos(finger.joint_names[1]),
    get_joint_pos(finger.joint_names[2]),
    get_joint_pos(finger.joint_names[3])
  };
}

void TactileGraspController::set_planar_q(int finger_idx, const std::array<double, 3> & q)
{
  auto & finger = fingers_[finger_idx];
  finger.current_joint_targets[1] = q[0];
  finger.current_joint_targets[2] = q[1];
  finger.current_joint_targets[3] = q[2];
}

bool TactileGraspController::apply_x_correction_by_ik(int finger_idx, bool forward_y, double cost)
{
  // thumb 제외
  if (finger_idx == 0) {
    return false;
  }

  if (!joint_received_ || !finger_planar_ik_) {
    return false;
  }

  const int solver_idx = finger_idx - 1;   // index=1 -> 0, ..., little=4 -> 3
  const auto current_q = get_planar_q(finger_idx);

  const double step_scale = finger_step_scale(finger_idx);
  const double delta_mag = ik_y_shift_base_ * std::max(cost, ik_min_cost_scale_) * step_scale;
  const double delta_y = forward_y ? delta_mag : -delta_mag;

  const auto maybe_q = finger_planar_ik_->solve_shift_y(solver_idx, current_q, delta_y);
  if (!maybe_q.has_value()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[%s] planar IK failed",
      fingers_[finger_idx].name.c_str());
    return false;
  }

  bool changed = false;
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(current_q[i] - maybe_q.value()[i]) > 1e-6) {
      changed = true;
      break;
    }
  }

  if (changed) {
    set_planar_q(finger_idx, maybe_q.value());
  }

  return changed;
}

void TactileGraspController::publish_traj()
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = this->now();
  traj_msg.joint_names = all_hand_joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;

  for (const auto & joint_name : all_hand_joint_names_) {
    double pos = 0.0;
    if (get_target(joint_name, pos)) {
      point.positions.push_back(pos);
    } else {
      point.positions.push_back(get_open_pos(joint_name));
    }
  }

  point.time_from_start = rclcpp::Duration::from_seconds(trajectory_dt_);
  traj_msg.points.push_back(point);
  traj_pub_->publish(traj_msg);
}

bool TactileGraspController::get_target(
  const std::string & joint_name, double & target) const
{
  for (const auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      if (finger.joint_names[j] == joint_name) {
        target = finger.current_joint_targets[j];
        return true;
      }
    }
  }
  return false;
}

double TactileGraspController::get_joint_pos(const std::string & joint_name) const
{
  auto it = joint_position_map_.find(joint_name);
  if (it != joint_position_map_.end()) {
    return it->second;
  }
  return 0.0;
}

double TactileGraspController::get_open_pos(const std::string & joint_name) const
{
  for (size_t i = 0; i < all_hand_joint_names_.size(); ++i) {
    if (all_hand_joint_names_[i] == joint_name) {
      return open_reference_positions_[i];
    }
  }
  return 0.0;
}

double TactileGraspController::clamp(double v, double min_v, double max_v) const
{
  return std::max(min_v, std::min(v, max_v));
}

std::string TactileGraspController::state_str(State s) const
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::CLOSE: return "CLOSE";
    case State::HOLD: return "HOLD";
    case State::OPEN: return "OPEN";
    default: return "UNKNOWN";
  }
}

std::string TactileGraspController::correction_str(CorrectionType t) const
{
  switch (t) {
    case CorrectionType::NONE: return "NONE";
    case CorrectionType::X_TOP: return "X_TOP";
    case CorrectionType::X_BOT: return "X_BOT";
    case CorrectionType::Y_LEFT: return "Y_LEFT";
    case CorrectionType::Y_RIGHT: return "Y_RIGHT";
    default: return "UNKNOWN";
  }
}

}  // namespace robotis_hand_tactile