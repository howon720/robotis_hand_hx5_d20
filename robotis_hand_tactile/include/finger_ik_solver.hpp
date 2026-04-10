#pragma once

#include <array>
#include <optional>

namespace robotis_hand_tactile {

class FingerPlanarIk
{
public:
  static constexpr int dof = 3;
  static constexpr int fingers_num = 4;   // index, middle, ring, little

  struct Pose2D
  {
    double y{0.0};
    double z{0.0};
    double theta{0.0};
  };

  struct FingerModel
  {
    std::array<double, dof> link_lengths{};
    std::array<double, dof> joint_min{};
    std::array<double, dof> joint_max{};
  };

  explicit FingerPlanarIk(const std::array<FingerModel, fingers_num> & models);

  Pose2D fk(int finger_idx, const std::array<double, dof> & q) const;

  std::optional<std::array<double, dof>> solve_shift_y(
    int finger_idx,
    const std::array<double, dof> & current_q,
    double delta_y) const;

private:
  std::array<FingerModel, fingers_num> models_;

  bool solve_numeric_ik(
    int finger_idx,
    const Pose2D & target,
    std::array<double, dof> & q) const;

  void clamp_to_limits(int finger_idx, std::array<double, dof> & q) const;
};

}  // namespace robotis_hand_tactile