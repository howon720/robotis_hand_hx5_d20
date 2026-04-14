#include "reactive_force.hpp"

#include <numeric>
#include <algorithm>
#include <cmath>

ReactiveForceNode::ReactiveForceNode()
    : Node("reactive_force"),
      step_count_(0) {
  force_topic_ = this->declare_parameter<std::string>(
      "force_topic", "/tactile_force");
  joint_state_topic_ = this->declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");
  traj_topic_ = this->declare_parameter<std::string>(
      "traj_topic", "/right_hand_controller/joint_trajectory");

  history_len_ = this->declare_parameter<int>("history_len", 3);
  startup_ignore_steps_ = this->declare_parameter<int>("startup_ignore_steps", 20); // 초반 몇 초 무시
  cooldown_steps_default_ = this->declare_parameter<int>("cooldown_steps", 5);      // release 사이 step
  traj_time_ = this->declare_parameter<double>("traj_time", 0.1);

  joint_min_ = this->declare_parameter<double>("joint_min", -1.5);
  joint_max_ = this->declare_parameter<double>("joint_max", 1.5);

  finger_names_ = {"thumb", "index", "middle", "ring", "little"};

  delta_threshold_["thumb"] = this->declare_parameter<double>("delta_threshold.thumb", 20.0);
  delta_threshold_["index"] = this->declare_parameter<double>("delta_threshold.index", 20.0);
  delta_threshold_["middle"] = this->declare_parameter<double>("delta_threshold.middle", 20.0);
  delta_threshold_["ring"] = this->declare_parameter<double>("delta_threshold.ring", 20.0);
  delta_threshold_["little"] = this->declare_parameter<double>("delta_threshold.little", 20.0);

  variation_threshold_["thumb"] = this->declare_parameter<double>("variation_threshold.thumb", 5.0);
  variation_threshold_["index"] = this->declare_parameter<double>("variation_threshold.index", 5.0);
  variation_threshold_["middle"] = this->declare_parameter<double>("variation_threshold.middle", 5.0);
  variation_threshold_["ring"] = this->declare_parameter<double>("variation_threshold.ring", 5.0);
  variation_threshold_["little"] = this->declare_parameter<double>("variation_threshold.little", 5.0);

  min_contact_force_["thumb"] = this->declare_parameter<double>("min_contact_force.thumb", 5.0);
  min_contact_force_["index"] = this->declare_parameter<double>("min_contact_force.index", 5.0);
  min_contact_force_["middle"] = this->declare_parameter<double>("min_contact_force.middle", 5.0);
  min_contact_force_["ring"] = this->declare_parameter<double>("min_contact_force.ring", 5.0);
  min_contact_force_["little"] = this->declare_parameter<double>("min_contact_force.little", 5.0);

  // release
  joint_release_step_["thumb"] = {
      {"finger_r_joint2", -0.1},
      {"finger_r_joint3", -0.1},
      {"finger_r_joint4", -0.1},
  };
  joint_release_step_["index"] = {
      {"finger_r_joint6", -0.1},
      {"finger_r_joint7", -0.1},
      {"finger_r_joint8", -0.1},
  };
  joint_release_step_["middle"] = {
      {"finger_r_joint10", -0.1},
      {"finger_r_joint11", -0.1},
      {"finger_r_joint12", -0.1},
  };
  joint_release_step_["ring"] = {
      {"finger_r_joint14", -0.1},
      {"finger_r_joint15", -0.1},
      {"finger_r_joint16", -0.1},
  };
  joint_release_step_["little"] = {
      {"finger_r_joint18", -0.1},
      {"finger_r_joint19", -0.1},
      {"finger_r_joint20", -0.1},
  };

  for (const auto& finger : finger_names_) {
    cooldown_counter_[finger] = 0;
  }

  force_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      force_topic_, 10, std::bind(&ReactiveForceNode::forceCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 50, std::bind(&ReactiveForceNode::jointStateCallback, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      traj_topic_, 10);

  grasp_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/grasp_state", 10, std::bind(&ReactiveForceNode::graspStateCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ReactiveForceNode started");
  RCLCPP_INFO(this->get_logger(), " force_topic      : %s", force_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " joint_state_topic: %s", joint_state_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " traj_topic       : %s", traj_topic_.c_str());
}

void ReactiveForceNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    current_joint_positions_[msg->name[i]] = msg->position[i];
  }
}

void ReactiveForceNode::graspStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  int prev_state = grasp_state_;
  grasp_state_ = msg->data;

  // moment ignore
  if (prev_state == 1 && grasp_state_ == 2) {
    post_grasp_ignore_counter_ = post_grasp_ignore_steps_;
  }

  for (auto& kv : force_history_) {
    kv.second.clear();
  }
}

double ReactiveForceNode::mean(const std::deque<double>& data) const {
  if (data.empty()) {
    return 0.0;
  }

  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  return sum / static_cast<double>(data.size());
}

double ReactiveForceNode::variation(const std::deque<double>& data) const {
  if (data.empty()) {
    return 0.0;
  }

  auto minmax = std::minmax_element(data.begin(), data.end());
  return (*minmax.second - *minmax.first);
}

double ReactiveForceNode::slope(const std::deque<double>& data) const {
  if (data.size() < 2) {
    return 0.0;
  }

  return data.back() - data.front();
}

void ReactiveForceNode::forceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  if (grasp_state_ == 1 || grasp_state_ == 3) {
    return;
  }

  if (post_grasp_ignore_counter_ > 0) {
    post_grasp_ignore_counter_--;
    return;
  }

  if (msg->data.size() < finger_names_.size() * 3) { // data 구조 바뀌면 여기 부분 수정
    RCLCPP_WARN(
        this->get_logger(),
        "Force array too small. expected=%zu, got=%zu",
        finger_names_.size(),
        msg->data.size());
    return;
  }

  step_count_++;

  std::vector<std::string> release_fingers;

  for (size_t i = 0; i < finger_names_.size(); ++i) {
    const auto& finger = finger_names_[i];
    size_t base = i * 3;

    double force = static_cast<double>(msg->data[base + 2]);

    auto& hist = force_history_[finger];
    hist.push_back(force);
    if (static_cast<int>(hist.size()) > history_len_) {
      hist.pop_front();
    }

    if (cooldown_counter_[finger] > 0) {
      cooldown_counter_[finger]--;
    }

    if (step_count_ < startup_ignore_steps_) {
      continue;
    }

    if (static_cast<int>(hist.size()) < history_len_) {
      continue;
    }

    if (cooldown_counter_[finger] > 0) {
      continue;
    }

    double avg = mean(hist);
    double var = variation(hist);
    double d = force - avg;
    double sl = slope(hist);

    // pub 조건
    bool trigger =
        (force > min_contact_force_[finger]) &&
        (d > delta_threshold_[finger]) &&
        (var > variation_threshold_[finger]); // &&
    //   (sl > delta_threshold_[finger] * 0.5);

    if (trigger) {
      release_fingers.push_back(finger);
      cooldown_counter_[finger] = cooldown_steps_default_;

      RCLCPP_INFO(
          this->get_logger(),
          "[RELEASE] %s force=%.2f avg=%.2f delta=%.2f var=%.2f slope=%.2f",
          finger.c_str(),
          force,
          avg,
          d,
          var,
          sl);
    }
  }

  if (!release_fingers.empty()) {
    publishReleaseTrajectory(release_fingers);
  }
}

void ReactiveForceNode::publishReleaseTrajectory(const std::vector<std::string>& release_fingers) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = this->now();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(traj_time_);

  for (const auto& finger : release_fingers) {
    auto finger_it = joint_release_step_.find(finger);
    if (finger_it == joint_release_step_.end()) {
      continue;
    }

    for (const auto& joint_pair : finger_it->second) {
      const std::string& joint_name = joint_pair.first;
      double step = joint_pair.second;

      auto joint_it = current_joint_positions_.find(joint_name);
      if (joint_it == current_joint_positions_.end()) {
        RCLCPP_WARN(
            this->get_logger(),
            "Current joint position not found for %s",
            joint_name.c_str());
        continue;
      }

      double current_pos = joint_it->second;
      double target_pos = std::clamp(current_pos + step, joint_min_, joint_max_);

      traj.joint_names.push_back(joint_name);
      point.positions.push_back(target_pos);
    }
  }

  if (traj.joint_names.empty()) {
    return;
  }

  point.velocities.resize(point.positions.size(), 0.0);
  point.accelerations.resize(point.positions.size(), 0.0);
  traj.points.push_back(point);

  traj_pub_->publish(traj);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReactiveForceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}