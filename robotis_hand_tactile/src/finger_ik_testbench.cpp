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

using namespace std::chrono_literals;

namespace robotis_hand_tactile
{

class FingerIkTestNode : public rclcpp::Node
{
public:
  FingerIkTestNode()
  : Node("finger_ik_test_node")
  {
    delta_y_ = this->declare_parameter<double>("delta_y", -0.0005);
    test_period_sec_ = this->declare_parameter<double>("test_period_sec", 1.0);

    // joint 이름
    // index  : joint6,7,8
    // middle : joint10,11,12
    // ring   : joint14,15,16
    // little : joint18,19,20
    finger_joint_names_[0] = {"finger_r_joint6",  "finger_r_joint7",  "finger_r_joint8"};
    finger_joint_names_[1] = {"finger_r_joint10", "finger_r_joint11", "finger_r_joint12"};
    finger_joint_names_[2] = {"finger_r_joint14", "finger_r_joint15", "finger_r_joint16"};
    finger_joint_names_[3] = {"finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};

    // 현재 네가 쓰는 link 길이 그대로
    std::array<FingerPlanarIk::FingerModel, 4> ik_models{};
    for (int i = 0; i < 4; ++i) {
      // 일단 테스트용 limit
      // 실제 limit 값을 알고 있으면 여기 숫자만 바꾸면 됨
      ik_models[i].joint_min = {-1.57, -1.57, -1.57};
      ik_models[i].joint_max = { 1.57,  1.57,  1.57};

      // link
      ik_models[i].link_lengths = {0.0235, 0.0355, 0.0355};
    }

    ik_solver_ = std::make_unique<FingerPlanarIk>(ik_models);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FingerIkTestNode::joint_state_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(test_period_sec_),
      std::bind(&FingerIkTestNode::run_test, this));

    RCLCPP_INFO(this->get_logger(), "finger_ik_test_node started");
    RCLCPP_INFO(this->get_logger(), "delta_y=%.6f, test_period_sec=%.3f", delta_y_, test_period_sec_);
  }

private:
  std::string arr_to_str(const std::array<double, 3> & q) const
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "[" << q[0] << ", " << q[1] << ", " << q[2] << "]";
    return oss.str();
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = msg;
  }

  bool get_finger_q_from_joint_state(
    int finger_idx,
    std::array<double, 3> & q_out) const
  {
    if (!latest_joint_state_) {
      return false;
    }

    const auto & names = latest_joint_state_->name;
    const auto & positions = latest_joint_state_->position;

    if (names.size() != positions.size()) {
      return false;
    }

    std::unordered_map<std::string, double> joint_map;
    for (size_t i = 0; i < names.size(); ++i) {
      joint_map[names[i]] = positions[i];
    }

    for (int j = 0; j < 3; ++j) {
      const auto & joint_name = finger_joint_names_[finger_idx][j];
      auto it = joint_map.find(joint_name);
      if (it == joint_map.end()) {
        return false;
      }
      q_out[j] = it->second;
    }

    return true;
  }

  void run_test()
  {
    if (!latest_joint_state_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for /joint_states...");
      return;
    }

    const std::array<std::string, 4> finger_names = {
      "index", "middle", "ring", "little"
    };

    RCLCPP_INFO(this->get_logger(), "===== Finger IK test =====");

    for (int finger_idx = 0; finger_idx < 4; ++finger_idx) {
      std::array<double, 3> q_cur{};
      if (!get_finger_q_from_joint_state(finger_idx, q_cur)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] failed to read joint states",
          finger_names[finger_idx].c_str());
        continue;
      }

      const auto pose_cur = ik_solver_->fk(finger_idx, q_cur);

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] current q = %s",
        finger_names[finger_idx].c_str(),
        arr_to_str(q_cur).c_str());

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] current pose: y=%.6f, z=%.6f, theta=%.6f",
        finger_names[finger_idx].c_str(),
        pose_cur.y, pose_cur.z, pose_cur.theta);

      auto q_sol = ik_solver_->solve_shift_y(finger_idx, q_cur, delta_y_);

      if (!q_sol) {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] IK failed",
          finger_names[finger_idx].c_str());
        continue;
      }

      const auto pose_new = ik_solver_->fk(finger_idx, *q_sol);

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] solved q = %s",
        finger_names[finger_idx].c_str(),
        arr_to_str(*q_sol).c_str());

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] new pose: y=%.6f, z=%.6f, theta=%.6f",
        finger_names[finger_idx].c_str(),
        pose_new.y, pose_new.z, pose_new.theta);

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] result: dy=%.6f, dz=%.6f",
        finger_names[finger_idx].c_str(),
        pose_new.y - pose_cur.y,
        pose_new.z - pose_cur.z);
    }
  }

private:
  double delta_y_{0.005};
  double test_period_sec_{1.0};

  std::unique_ptr<FingerPlanarIk> ik_solver_;

  std::array<std::array<std::string, 3>, 4> finger_joint_names_{};

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robotis_hand_tactile

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotis_hand_tactile::FingerIkTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}