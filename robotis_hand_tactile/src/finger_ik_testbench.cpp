#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "finger_ik_solver.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

namespace robotis_hand_tactile {

class FingerIkTestNode : public rclcpp::Node {
public:
  FingerIkTestNode()
      : Node("finger_ik_test_node") // 0.001 0.001
  {
    delta_y_ = this->declare_parameter<double>("delta_y", 0.000);
    delta_z_ = this->declare_parameter<double>("delta_z", 0.002);
    test_period_sec_ = this->declare_parameter<double>("test_period_sec", 1.0);
    move_time_sec_ = this->declare_parameter<double>("move_time_sec", 1.0);

    // /joint_states : rad

    finger_joint_names_[0] = {"finger_r_joint6", "finger_r_joint7", "finger_r_joint8"};
    finger_joint_names_[1] = {"finger_r_joint10", "finger_r_joint11", "finger_r_joint12"};
    finger_joint_names_[2] = {"finger_r_joint14", "finger_r_joint15", "finger_r_joint16"};
    finger_joint_names_[3] = {"finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};

    all_joint_names_ = {
        "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4", "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8", "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12", "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16", "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};

    std::array<FingerPlanarIk::FingerModel, 4> ik_models{};
    for (int i = 0; i < 4; ++i) {
      ik_models[i].joint_min = {-1.5, -1.5, -1.5};
      ik_models[i].joint_max = {1.5, 1.5, 1.5};

      ik_models[i].link_lengths = {0.0235, 0.0355, 0.0355};
    }

    ik_solver_ = std::make_unique<FingerPlanarIk>(ik_models);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&FingerIkTestNode::joint_state_callback, this, std::placeholders::_1));

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/right_hand_controller/joint_trajectory", 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(test_period_sec_),
        std::bind(&FingerIkTestNode::run_test, this));

    RCLCPP_INFO(this->get_logger(), "finger_ik_test_node started");
    RCLCPP_INFO(
        this->get_logger(),
        "joint values are interpreted as relative joint angles in rad");
  }

private:
  std::string arr_to_str(const std::array<double, 3>& q) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "[" << q[0] << ", " << q[1] << ", " << q[2] << "]";
    return oss.str();
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_state_ = msg;
  }

  bool build_joint_map(std::unordered_map<std::string, double>& joint_map) const {
    if (!latest_joint_state_) {
      return false;
    }

    const auto& names = latest_joint_state_->name;
    const auto& positions = latest_joint_state_->position;

    if (names.size() != positions.size()) {
      return false;
    }

    joint_map.clear();
    for (size_t i = 0; i < names.size(); ++i) {
      joint_map[names[i]] = positions[i];
    }

    return true;
  }

  bool get_finger_q_from_joint_state(
      int finger_idx,
      const std::unordered_map<std::string, double>& joint_map,
      std::array<double, 3>& q_out) const {
    for (int j = 0; j < 3; ++j) {
      const auto& joint_name = finger_joint_names_[finger_idx][j];
      auto it = joint_map.find(joint_name);
      if (it == joint_map.end()) {
        return false;
      }

      // rad 값
      q_out[j] = it->second;
    }

    return true;
  }

  bool build_full_target_positions(
      const std::unordered_map<std::string, double>& joint_map,
      const std::array<std::array<double, 3>, 4>& q_targets,
      const std::array<bool, 4>& ik_success,
      std::vector<double>& positions_out) const {
    positions_out.clear();
    positions_out.reserve(all_joint_names_.size());

    // 전체 조인트는 현재값으로 초기화
    for (const auto& joint_name : all_joint_names_) {
      auto it = joint_map.find(joint_name);
      if (it == joint_map.end()) {
        return false;
      }
      positions_out.push_back(it->second);
    }

    // 성공한 finger만 IK 결과로 덮기
    for (int finger_idx = 0; finger_idx < 4; ++finger_idx) {
      if (!ik_success[finger_idx]) {
        continue;
      }

      for (int j = 0; j < 3; ++j) {
        const auto& target_joint_name = finger_joint_names_[finger_idx][j];

        for (size_t i = 0; i < all_joint_names_.size(); ++i) {
          if (all_joint_names_[i] == target_joint_name) {
            positions_out[i] = q_targets[finger_idx][j];
            break;
          }
        }
      }
    }

    return true;
  }

  void publish_trajectory(const std::vector<double>& target_positions) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = all_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_positions;
    point.time_from_start.sec = static_cast<int32_t>(move_time_sec_);
    point.time_from_start.nanosec =
        static_cast<uint32_t>((move_time_sec_ - std::floor(move_time_sec_)) * 1e9);

    traj.points.push_back(point);

    traj_pub_->publish(traj);

    RCLCPP_INFO(
        this->get_logger(),
        "published trajectory to /right_hand_controller/joint_trajectory");
  }

  void run_test() {
    if (!latest_joint_state_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "waiting for /joint_states...");
      return;
    }

    std::unordered_map<std::string, double> joint_map;
    if (!build_joint_map(joint_map)) {
      RCLCPP_ERROR(this->get_logger(), "failed to build joint_map");
      return;
    }

    const std::array<std::string, 4> finger_names = {
        "index", "middle", "ring", "little"};

    std::array<std::array<double, 3>, 4> q_targets{};
    std::array<bool, 4> ik_success{false, false, false, false};

    RCLCPP_INFO(this->get_logger(), "===== Finger IK test =====");

    for (int finger_idx = 0; finger_idx < 4; ++finger_idx) {
      std::array<double, 3> q_cur{};
      if (!get_finger_q_from_joint_state(finger_idx, joint_map, q_cur)) {
        RCLCPP_WARN(
            this->get_logger(),
            "[%s] failed to read joint states",
            finger_names[finger_idx].c_str());
        continue;
      }

      const auto pose_cur = ik_solver_->fk(finger_idx, q_cur);

      const double target_y = pose_cur.y + delta_y_;
      const double target_z = pose_cur.z + delta_z_;

      RCLCPP_INFO(
          this->get_logger(),
          "[%s] current q(rad) = %s",
          finger_names[finger_idx].c_str(),
          arr_to_str(q_cur).c_str());

      RCLCPP_INFO(
          this->get_logger(),
          "[%s] current pose: y=%.6f, z=%.6f, theta=%.6f",
          finger_names[finger_idx].c_str(),
          pose_cur.y,
          pose_cur.z,
          pose_cur.theta);

      RCLCPP_INFO(
          this->get_logger(),
          "[%s] target_y=%.6f target_z=%.6f (delta_y=%.6f, delta_z=%.6f)",
          finger_names[finger_idx].c_str(),
          target_y,
          target_z,
          delta_y_,
          delta_z_);

      auto q_sol = ik_solver_->solve_shift_yz(finger_idx, q_cur, delta_y_, delta_z_);

      if (!q_sol) {
        RCLCPP_WARN(
            this->get_logger(),
            "[%s] IK failed -> keep current joint values",
            finger_names[finger_idx].c_str());

        q_targets[finger_idx] = q_cur;
        ik_success[finger_idx] = false;
        continue;
      }

      const auto pose_new = ik_solver_->fk(finger_idx, *q_sol);

      RCLCPP_INFO(
          this->get_logger(),
          "[%s] solved q(rad) = %s",
          finger_names[finger_idx].c_str(),
          arr_to_str(*q_sol).c_str());

      RCLCPP_INFO(
          this->get_logger(),
          "[%s] new pose: y=%.6f, z=%.6f, theta=%.6f",
          finger_names[finger_idx].c_str(),
          pose_new.y,
          pose_new.z,
          pose_new.theta);

      RCLCPP_INFO(
          this->get_logger(),
          "[%s] result: dy=%.6f, dz=%.6f",
          finger_names[finger_idx].c_str(),
          pose_new.y - pose_cur.y,
          pose_new.z - pose_cur.z);

      q_targets[finger_idx] = *q_sol;
      ik_success[finger_idx] = true;
    }

    std::vector<double> target_positions;
    if (!build_full_target_positions(joint_map, q_targets, ik_success, target_positions)) {
      RCLCPP_ERROR(this->get_logger(), "failed to build full target positions");
      return;
    }

    publish_trajectory(target_positions);
  }

private:
  double delta_y_{0.000};
  double delta_z_{0.000};
  double test_period_sec_{1.0};
  double move_time_sec_{1.0};

  std::unique_ptr<FingerPlanarIk> ik_solver_;
  std::array<std::array<std::string, 3>, 4> finger_joint_names_{};
  std::vector<std::string> all_joint_names_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace robotis_hand_tactile

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotis_hand_tactile::FingerIkTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}