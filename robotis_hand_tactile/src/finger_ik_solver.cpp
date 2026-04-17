#include "finger_ik_solver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace robotis_hand_tactile {

namespace {
double clamp_value(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
} // namespace

FingerPlanarIk::FingerPlanarIk(const std::array<FingerModel, fingers_num>& models) : models_(models) {
}

FingerPlanarIk::Pose2D FingerPlanarIk::fk(int finger_idx, const std::array<double, dof>& q) const {
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

void FingerPlanarIk::clamp_to_limits(int finger_idx, std::array<double, dof>& q) const {
  for (int i = 0; i < dof; ++i) {
    q[i] = clamp_value(q[i], models_[finger_idx].joint_min[i], models_[finger_idx].joint_max[i]);
  }
}

std::optional<std::array<double, FingerPlanarIk::dof>> FingerPlanarIk::solve_shift_yz(
    int finger_idx, const std::array<double, dof>& current_q, double delta_y, double delta_z) const {
  const Pose2D current_pose = fk(finger_idx, current_q);

  Pose2D target = current_pose;
  target.y += delta_y;
  target.z += delta_z;

  return solve_ik(finger_idx, target, current_q);
}

std::optional<std::array<double, FingerPlanarIk::dof>>
FingerPlanarIk::solve_ik(int finger_idx, const Pose2D& target, const std::array<double, dof>& current_q) const {
  const double l1 = models_[finger_idx].link_lengths[0];
  const double l2 = models_[finger_idx].link_lengths[1];
  const double l3 = models_[finger_idx].link_lengths[2];

  const double q1c = current_q[0];
  const double q2c = current_q[1];
  const double q3c = current_q[2];

  // q3 = q3c + (q1-q1c) + (q2-q2c)
  const double c = q3c - q1c - q2c;

  auto build_q = [&](double q1, double q2) {
    std::array<double, dof> q{};
    q[0] = q1;
    q[1] = q2;
    q[2] = q3c + (q1 - q1c) + (q2 - q2c);
    return q;
  };

  auto residual = [&](double q1, double q2, double& ey, double& ez) {
    const double a1 = q1;
    const double a2 = q1 + q2;
    const double a3 = 2.0 * (q1 + q2) + c; // q1 + q2 + q3

    const double y = l1 * std::sin(a1) + l2 * std::sin(a2) + l3 * std::sin(a3);

    const double z = l1 * std::cos(a1) + l2 * std::cos(a2) + l3 * std::cos(a3);

    ey = target.y - y;
    ez = target.z - z;
  };

  // low residual sol
  const std::vector<std::array<double, 2>> seeds = {{q1c, q2c},
                                                    {q1c + 0.05, q2c},
                                                    {q1c - 0.05, q2c},
                                                    {q1c, q2c + 0.05},
                                                    {q1c, q2c - 0.05},
                                                    {q1c + 0.05, q2c + 0.05},
                                                    {q1c - 0.05, q2c - 0.05}};

  std::optional<std::array<double, dof>> best_q;
  double best_cost = std::numeric_limits<double>::infinity();

  for (const auto& seed : seeds) {
    double q1 = seed[0];
    double q2 = seed[1];

    for (int iter = 0; iter < 50; ++iter) {
      double ey = 0.0, ez = 0.0;
      residual(q1, q2, ey, ez);

      const double a1 = q1;
      const double a2 = q1 + q2;
      const double a3 = 2.0 * (q1 + q2) + c;

      // exact Jacobian for constrained model
      const double j11 = l1 * std::cos(a1) + l2 * std::cos(a2) + 2.0 * l3 * std::cos(a3);
      const double j12 = l2 * std::cos(a2) + 2.0 * l3 * std::cos(a3);

      const double j21 = -l1 * std::sin(a1) - l2 * std::sin(a2) - 2.0 * l3 * std::sin(a3);
      const double j22 = -l2 * std::sin(a2) - 2.0 * l3 * std::sin(a3);

      const double det = j11 * j22 - j12 * j21;
      if (std::abs(det) < 1e-10) {
        break;
      }

      // Newton step: J * dq = e
      const double dq1 = (j22 * ey - j12 * ez) / det;
      const double dq2 = (-j21 * ey + j11 * ez) / det;

      q1 += dq1;
      q2 += dq2;

      auto q_tmp = build_q(q1, q2);
      clamp_to_limits(finger_idx, q_tmp);
      q1 = q_tmp[0];
      q2 = q_tmp[1];
    }

    auto q_candidate = build_q(q1, q2);
    clamp_to_limits(finger_idx, q_candidate);

    // limit clamp 이후 residual checks
    double ey = 0.0, ez = 0.0;
    const auto p_chk = fk(finger_idx, q_candidate);
    ey = target.y - p_chk.y;
    ez = target.z - p_chk.z;

    const double pos_err = std::sqrt(ey * ey + ez * ez);
    const double joint_change =
        std::abs(q_candidate[0] - q1c) + std::abs(q_candidate[1] - q2c) + std::abs(q_candidate[2] - q3c);

    // best_path: low residual + low regularization
    const double cost = pos_err + 0.01 * joint_change;

    if (cost < best_cost) {
      best_cost = cost;
      best_q = q_candidate;
    }
  }

  if (!best_q) {
    return std::nullopt;
  }

  // too far : failed
  const auto p_final = fk(finger_idx, *best_q);
  const double final_err =
      std::sqrt((target.y - p_final.y) * (target.y - p_final.y) + (target.z - p_final.z) * (target.z - p_final.z));

  if (final_err > 5e-4) {
    return std::nullopt;
  }

  return best_q;
}

} // namespace robotis_hand_tactile