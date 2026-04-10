#include "finger_ik_solver.hpp"
#include <cmath>

namespace robotis_hand_tactile {

FingerPlanarIk::FingerPlanarIk(const std::array<FingerModel, fingers_num> & models)
: models_(models)
{
}

FingerPlanarIk::Pose2D FingerPlanarIk::fk(
  int finger_idx, const std::array<double, dof> & q) const
{
  Pose2D pose;
  double angle = 0.0;

  for (int i = 0; i < dof; ++i) {
    angle += q[i];
    pose.y += models_[finger_idx].link_lengths[i] * std::sin(angle);
    pose.z += models_[finger_idx].link_lengths[i] * std::cos(angle);
  }

  pose.theta = angle;
  return pose;
}

void FingerPlanarIk::clamp_to_limits(
  int finger_idx, std::array<double, dof> & q) const
{
  for (int i = 0; i < dof; ++i) {
    q[i] = std::max(models_[finger_idx].joint_min[i],
           std::min(models_[finger_idx].joint_max[i], q[i]));
  }
}

std::optional<std::array<double, FingerPlanarIk::dof>>
FingerPlanarIk::solve_shift_y(
  int finger_idx,
  const std::array<double, dof> & current_q,
  double delta_y) const
{
  std::array<double, dof> q = current_q;
  const Pose2D current_pose = fk(finger_idx, current_q);

  Pose2D target = current_pose;
  target.y += delta_y;

  if (!solve_numeric_ik(finger_idx, target, q)) {
    return std::nullopt;
  }

  clamp_to_limits(finger_idx, q);
  return q;
}

bool FingerPlanarIk::solve_numeric_ik(
  int finger_idx,
  const Pose2D & target,
  std::array<double, dof> & q) const
{
  constexpr int max_iter = 60;
  constexpr double alpha = 0.2;
  constexpr double tol = 1e-4;

  for (int iter = 0; iter < max_iter; ++iter) {
    const Pose2D p = fk(finger_idx, q);
    const double ey = target.y - p.y;
    const double ez = target.z - p.z;
    const double err = std::sqrt(ey * ey + ez * ez);

    if (err < tol) {
      return true;
    }

    std::array<double, dof> dpy{};
    std::array<double, dof> dpz{};

    for (int j = 0; j < dof; ++j) {
      double angle_sum = 0.0;
      double jy = 0.0;
      double jz = 0.0;

      for (int k = 0; k < dof; ++k) {
        angle_sum += q[k];
        if (k >= j) {
          jy += models_[finger_idx].link_lengths[k] * std::cos(angle_sum);
          jz += -models_[finger_idx].link_lengths[k] * std::sin(angle_sum);
        }
      }

      dpy[j] = jy;
      dpz[j] = jz;
    }

    for (int j = 0; j < dof; ++j) {
      const double dq = alpha * (dpy[j] * ey + dpz[j] * ez);
      q[j] += dq;
    }

    clamp_to_limits(finger_idx, q);
  }

  return false;
}

}  // namespace robotis_hand_tactile