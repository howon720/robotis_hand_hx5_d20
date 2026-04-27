#include "tactile_grasp_controller_node.hpp"

using namespace std::chrono_literals;

namespace robotis_hand_tactile {

TactileGraspControllerNode::TactileGraspControllerNode()
    : TactileGraspController("tactile_grasp_controller"), tactile_sensor_(this->get_logger(), this->get_clock()) {

  // Load parameters.
  declare_params(this);
  param = load_params(this);
  tactile_sensor_.set_params(param);

  // Initialize ROS subscriptions.
  pressure_sub_ = this->create_subscription<HandPressuresMsg>(
      "/right_hand/finger_pressures",
      10,
      std::bind(&TactileGraspControllerNode::on_pressure, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<JointStateMsg>(
      "/joint_states", 10, std::bind(&TactileGraspControllerNode::on_joint_state, this, std::placeholders::_1));

  grasp_state_sub_ = this->create_subscription<Int32Msg>(
      "/grasp_state", 10, std::bind(&TactileGraspControllerNode::on_grasp_state, this, std::placeholders::_1));

  // Initialize ROS publishers.
  traj_pub_ = this->create_publisher<JointTrajectoryMsg>("/right_hand_controller/joint_trajectory", 10);

  // Start main control loop.
  const auto period = std::chrono::duration<double>(1.0 / std::max(param.control_hz, 1.0));
  control_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                           std::bind(&TactileGraspControllerNode::control_loop, this));

  // Move unused fingers independently.
  unused_finger_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    close_unused_finger();
    publish_traj();
  });
  RCLCPP_INFO(this->get_logger(), "TactileGraspController initialized.");
}

void TactileGraspControllerNode::on_pressure(const HandPressuresPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!tactile_sensor_.check_msg(msg)) {
    return;
  }

  // Store current joint positions by joint name.
  const auto sensors = tactile_sensor_.parse_sensors(msg);
  tactile_sensor_.update_pressure(fingers_, baseline_, sensors);
}

void TactileGraspControllerNode::on_joint_state(const JointStatePtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  curr_joint_.clear();

  // Store current joint positions by joint name.
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    curr_joint_[msg->name[i]] = msg->position[i];
  }
  joint_received_ = true;
}

void TactileGraspControllerNode::on_grasp_state(const Int32Ptr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Start grasping when grasp_state is 3.
  if (msg->data == 3) {
    if (state_ == State::IDLE) {
      reset_grasp();
      state_ = State::CLOSE;
      RCLCPP_INFO(this->get_logger(), "Start Grasping");
    }
  }
}

void TactileGraspControllerNode::control_loop() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Run state-specific controller logic.
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
}

void TactileGraspControllerNode::publish_traj() {
  JointTrajectoryMsg traj_msg;
  JointTrajectoryPointMsg point;
  traj_msg.header.stamp = this->now();
  traj_msg.joint_names = hand_joint_names_;

  // Fill trajectory point with current target values.
  for (const auto& joint_name : hand_joint_names_) {
    double pos = 0.0;
    if (get_target(joint_name, pos)) {
      point.positions.push_back(pos);
    } else {
      point.positions.push_back(get_open_pos(joint_name));
    }
  }
  point.time_from_start = rclcpp::Duration::from_seconds(param.trajectory_dt);
  traj_msg.points.push_back(point);
  traj_pub_->publish(traj_msg);
}

std::optional<CorrectionDecision> TactileGraspControllerNode::pick_correction(const CopInfo& info) const {
  return tactile_sensor_.pick_correction(info);
}

} // namespace robotis_hand_tactile

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotis_hand_tactile::TactileGraspControllerNode>());
  rclcpp::shutdown();
  return 0;
}