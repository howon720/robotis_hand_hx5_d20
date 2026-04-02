#ifndef ROBOTIS_HAND_TACTILE__TACTILE_FORCE_RVIZ_HPP_
#define ROBOTIS_HAND_TACTILE__TACTILE_FORCE_RVIZ_HPP_

#include <array>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

class TactileForceRviz : public rclcpp::Node
{
public:
  TactileForceRviz();

private:
  void init_taxel_positions();
  int finger_index_from_joint(const std::string & joint_name) const;
  std::vector<double> extract_pressures(
    const control_msgs::msg::InterfaceValue & iv) const;

  std::array<double, 3> map_sensor_vector_to_link(
    int finger_idx, double sx, double sy, double sn) const;

  void callback(const control_msgs::msg::DynamicJointState::SharedPtr msg);
  void accumulate_baseline(int finger_idx, const std::vector<double> & vals);
  void update_pressure(int finger_idx, const std::vector<double> & vals);
  void finalize_baseline();

  std::array<double, 3> compute_force_vector(
    int finger_idx, const std::vector<double> & p) const;

  visualization_msgs::msg::Marker make_arrow_marker(
    int finger_idx, const std::array<double, 3> & vec) const;

  void publish_markers();

  double compute_total_force(const std::vector<double> & p) const;

private:
  std::string topic_;
  std::string sensor_prefix_;
  int num_fingers_;
  int num_taxels_;
  std::string pressure_iface_prefix_;
  double update_hz_;
  bool use_best_effort_;

  double baseline_seconds_;
  double ema_alpha_;
  double deadband_;
  bool clip_negative_;

  std::string marker_topic_;
  std::string marker_ns_;
  std::vector<std::string> finger_frames_;

  double taxel_pitch_x_;
  double taxel_pitch_y_;

  double lateral_gain_;
  double normal_gain_;

  double force_to_arrow_scale_;
  double min_arrow_len_;
  double max_arrow_len_;
  double shaft_diameter_;
  double head_diameter_;
  double head_length_;

  double normal_sign_;

  int baseline_frames_;
  int baseline_count_;
  bool baseline_ready_;

  std::vector<std::pair<double, double>> taxel_xy_;

  std::vector<std::vector<double>> pressure_;
  std::vector<std::vector<double>> ema_;
  std::vector<std::vector<double>> baseline_;
  std::vector<std::vector<double>> baseline_sum_;
  std::vector<int> baseline_samples_per_finger_;

  std::mutex mutex_;

  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr force_pub_;  // 힘 pub
};

#endif  // ROBOTIS_HAND_TACTILE__TACTILE_FORCE_RVIZ_HPP_