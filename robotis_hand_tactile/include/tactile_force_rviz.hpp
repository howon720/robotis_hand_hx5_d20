#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "robotis_interfaces/msg/tactile_sensor.hpp"

#include <array>
#include <vector>
#include <string>
#include <mutex>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cctype>

class TactileForceRviz : public rclcpp::Node {
public:
  TactileForceRviz();

private:
  typedef robotis_interfaces::msg::HandPressures HandPressures;
  typedef robotis_interfaces::msg::TactileSensor TactileSensor;
  typedef std::array<std::array<float, 4>, 5> ColorArray;

  enum DirectionRegion {
    CENTER = 0,
    UP = 1,
    DOWN = 2,
    LEFT = 3,
    RIGHT = 4
  };

  struct DirectionInfo {
    double total_force{0.0};
    double cop_x{0.0};
    double cop_y{0.0};
    double angle_rad{0.0};
    int region{CENTER};
    std::array<double, 3> vec{0.0, 0.0, 0.0};
  };

  void callback(const HandPressures::SharedPtr msg);
  void publish_markers();

  void init_taxel_positions();
  int finger_index_from_sensor(const std::string& sensor_name) const;
  std::vector<double> extract_pressures(const TactileSensor& sensor_msg) const;

  void accumulate_baseline(int finger_idx, const std::vector<double>& vals);
  void update_pressure(int finger_idx, const std::vector<double>& vals);
  void finalize_baseline();

  double compute_total_force(const std::vector<double>& p) const;

  std::array<double, 3> map_sensor_vector_to_link(int finger_idx, double sx, double sy, double sn) const;

  std::array<double, 3> compute_force_vector(int finger_idx, const std::vector<double>& p) const;

  int classify_region(double cop_x, double cop_y, double eps_x, double eps_y) const;
  DirectionInfo compute_direction_info(int finger_idx, const std::vector<double>& p) const;

  geometry_msgs::msg::Point cop_point_in_frame(int finger_idx, const DirectionInfo& info) const;

  visualization_msgs::msg::Marker make_arrow_marker(int finger_idx, const DirectionInfo& info) const;

  visualization_msgs::msg::Marker make_cop_marker(int finger_idx, const DirectionInfo& info) const;

private:
  std::string topic_;
  std::string sensor_prefix_;
  int num_fingers_;
  int num_taxels_;
  double update_hz_;
  bool use_best_effort_;

  double baseline_seconds_;
  double ema_alpha_;
  double deadband_;
  bool clip_negative_;

  std::string marker_topic_;
  std::string marker_ns_;

  double center_region_ratio_;

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

  double cop_marker_offset_;
  double cop_marker_scale_;

  std::vector<std::vector<double>> pressure_;
  std::vector<std::vector<double>> ema_;
  std::vector<std::vector<double>> baseline_;
  std::vector<std::vector<double>> baseline_sum_;

  std::vector<std::pair<double, double>> taxel_xy_;

  int baseline_count_;
  bool baseline_ready_;
  int baseline_frames_;
  std::vector<int> baseline_samples_per_finger_;

  std::mutex mutex_;

  rclcpp::Subscription<HandPressures>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr force_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};