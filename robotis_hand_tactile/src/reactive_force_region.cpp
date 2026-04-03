#include "reactive_force_region.hpp"

#include <numeric>
#include <algorithm>
#include <cmath>

ReactiveForceNode::ReactiveForceNode()
: Node("reactive_force"),
  step_count_(0)
{
  force_topic_ = this->declare_parameter<std::string>(
    "force_topic", "/tactile_force");
  joint_state_topic_ = this->declare_parameter<std::string>(
    "joint_state_topic", "/joint_states");
  traj_topic_ = this->declare_parameter<std::string>(
    "traj_topic", "/right_hand_controller/joint_trajectory");

  history_len_ = this->declare_parameter<int>("history_len", 3);
  startup_ignore_steps_ = this->declare_parameter<int>("startup_ignore_steps", 20);
  cooldown_steps_default_ = this->declare_parameter<int>("cooldown_steps", 5);
  traj_time_ = this->declare_parameter<double>("traj_time", 0.1);

  joint_min_ = this->declare_parameter<double>("joint_min", -1.5);
  joint_max_ = this->declare_parameter<double>("joint_max", 1.5);

  delta_threshold_ = this->declare_parameter<double>("delta_threshold", 2.0);
  variation_threshold_ = this->declare_parameter<double>("variation_threshold", 1.0);
  min_contact_force_ = this->declare_parameter<double>("min_contact_force", 1.0);

  post_grasp_ignore_steps_ = this->declare_parameter<int>("post_grasp_ignore_steps", 50);

  finger_names_ = {"thumb", "index", "middle", "ring", "little"};

  // 1:UP 2:DOWN 3:LEFT 4:RIGHT
  joint_release_step_["thumb"] = {
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

  joint_region34_step_["thumb"]  = {"finger_r_joint2",  0.1};
  joint_region34_step_["index"]  = {"finger_r_joint5",  0.1};
  joint_region34_step_["middle"] = {"finger_r_joint9",  0.1};
  joint_region34_step_["ring"]   = {"finger_r_joint13", 0.1};
  joint_region34_step_["little"] = {"finger_r_joint17", 0.1};

  for (const auto & finger : finger_names_) {
    cooldown_counter_[finger] = 0;
  }

  force_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    force_topic_, 10,
    std::bind(&ReactiveForceNode::forceCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, 50,
    std::bind(&ReactiveForceNode::jointStateCallback, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    traj_topic_, 10);

  grasp_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/grasp_state", 10,
    std::bind(&ReactiveForceNode::graspStateCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ReactiveForceNode started");
  RCLCPP_INFO(this->get_logger(), " force_topic      : %s", force_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " joint_state_topic: %s", joint_state_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), " traj_topic       : %s", traj_topic_.c_str());
}

void ReactiveForceNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    current_joint_positions_[msg->name[i]] = msg->position[i];
  }
}

void ReactiveForceNode::graspStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  int prev_state = grasp_state_;
  grasp_state_ = msg->data;

  if (prev_state == 1 && grasp_state_ == 2) {
    post_grasp_ignore_counter_ = post_grasp_ignore_steps_;
  }

  for (auto & kv : force_history_) {
    kv.second.clear();
  }
}

double ReactiveForceNode::mean(const std::deque<double> & data) const
{
  if (data.empty()) {
    return 0.0;
  }

  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  return sum / static_cast<double>(data.size());
}

double ReactiveForceNode::variation(const std::deque<double> & data) const
{
  if (data.empty()) {
    return 0.0;
  }

  auto minmax = std::minmax_element(data.begin(), data.end());
  return (*minmax.second - *minmax.first);
}

double ReactiveForceNode::slope(const std::deque<double> & data) const
{
  if (data.size() < 2) {
    return 0.0;
  }

  return data.back() - data.front();
}

void ReactiveForceNode::forceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (grasp_state_ == 1 || grasp_state_ == 3) {
    return;
  }

  if (post_grasp_ignore_counter_ > 0) {
    post_grasp_ignore_counter_--;
    return;
  }

  const size_t values_per_finger = 3;
  const size_t expected_size = finger_names_.size() * values_per_finger;

  if (msg->data.size() < expected_size) {
    RCLCPP_WARN(
      this->get_logger(),
      "Force array too small. expected=%zu, got=%zu",
      expected_size, msg->data.size());
    return;
  }

  step_count_++;

  std::vector<std::pair<std::string, int>> release_fingers;

  for (size_t i = 0; i < finger_names_.size(); ++i) {
    const auto & finger = finger_names_[i];
    const size_t base = i * values_per_finger;

    const int region = static_cast<int>(msg->data[base + 0]);
    const double angle_deg = static_cast<double>(msg->data[base + 1]);
    const double force = static_cast<double>(msg->data[base + 2]);

    (void)angle_deg;

    auto & hist = force_history_[finger];
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

    if (region < 1 || region > 4) {
      continue;
    }

    const double avg = mean(hist);
    const double var = variation(hist);
    const double d = force - avg;
    const double sl = slope(hist);

    const bool trigger =
      (force > min_contact_force_) &&
      (d > delta_threshold_) &&
      (var > variation_threshold_);

    if (trigger) {
      release_fingers.push_back({finger, region});
      cooldown_counter_[finger] = cooldown_steps_default_;

      RCLCPP_INFO(
        this->get_logger(),
        "[RELEASE] %s region=%d force=%.2f avg=%.2f delta=%.2f var=%.2f slope=%.2f",
        finger.c_str(), region, force, avg, d, var, sl);
    }
  }

  if (!release_fingers.empty()) {
    publishReleaseTrajectory(release_fingers);
  }
}

void ReactiveForceNode::publishReleaseTrajectory(
  const std::vector<std::pair<std::string, int>> & release_fingers)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = this->now();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(traj_time_);

  for (const auto & item : release_fingers) {
    const std::string & finger = item.first;
    const int region = item.second;

    // region 1,2: 기존 3개 joint 사용
    if (region == 1 || region == 2) {
      auto finger_it = joint_release_step_.find(finger);
      if (finger_it == joint_release_step_.end()) {
        continue;
      }

      const double direction_scale = (region == 1) ? 1.0 : -1.0;

      for (const auto & joint_pair : finger_it->second) {
        const std::string & joint_name = joint_pair.first;
        const double step = joint_pair.second * direction_scale;

        auto joint_it = current_joint_positions_.find(joint_name);
        if (joint_it == current_joint_positions_.end()) {
          RCLCPP_WARN(
            this->get_logger(),
            "Current joint position not found for %s",
            joint_name.c_str());
          continue;
        }

        const double current_pos = joint_it->second;
        const double target_pos = std::clamp(current_pos + step, joint_min_, joint_max_);

        traj.joint_names.push_back(joint_name);
        point.positions.push_back(target_pos);
      }
    }

    // region 3,4: 손가락별 joint 1개만 사용
    else if (region == 3 || region == 4) {
      auto finger_it = joint_region34_step_.find(finger);
      if (finger_it == joint_region34_step_.end()) {
        continue;
      }

      const std::string & joint_name = finger_it->second.first;
      const double base_step = finger_it->second.second;
      const double direction_scale = (region == 3) ? 1.0 : -1.0;
      const double step = base_step * direction_scale;

      auto joint_it = current_joint_positions_.find(joint_name);
      if (joint_it == current_joint_positions_.end()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Current joint position not found for %s",
          joint_name.c_str());
        continue;
      }

      const double current_pos = joint_it->second;
      const double target_pos = std::clamp(current_pos + step, joint_min_, joint_max_);

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReactiveForceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}