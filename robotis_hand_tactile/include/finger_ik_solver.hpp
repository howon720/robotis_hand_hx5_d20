#pragma once

#include <array>
#include <optional>

namespace robotis_hand_tactile {

class FingerPlanarIk {
public:
  static constexpr int fingers_num = 4;
  static constexpr int dof = 3;

  struct FingerModel {
    std::array<double, dof> joint_min{};
    std::array<double, dof> joint_max{};
    std::array<double, dof> link_lengths{};
  };

  struct Pose2D {
    double y{0.0};
    double z{0.0};
    double theta{0.0};
  };

  explicit FingerPlanarIk(const std::array<FingerModel, fingers_num>& models);

  Pose2D fk(int finger_idx, const std::array<double, dof>& q) const;

  void clamp_to_limits(int finger_idx, std::array<double, dof>& q) const;

  std::optional<std::array<double, dof>>
  solve_shift_yz(int finger_idx, const std::array<double, dof>& current_q, double delta_y, double delta_z) const;

  std::optional<std::array<double, dof>>
  solve_shift_local(int finger_idx, const std::array<double, dof>& current_q, double local_dy, double local_dz) const;

private:
  std::optional<std::array<double, dof>>
  solve_ik(int finger_idx, const Pose2D& target, const std::array<double, dof>& current_q) const;

  std::array<FingerModel, fingers_num> models_;
};

} // namespace robotis_hand_tactile