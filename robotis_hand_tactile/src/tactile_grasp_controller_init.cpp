#include "tactile_grasp_controller.hpp"

namespace robotis_hand_tactile {

void TactileGraspController::init_fingers() {
  // thumb
  fingers_[0].name = "thumb";
  fingers_[0].joint_names = {"finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4"};
  fingers_[0].joint_min = {-1.5, -3.5, -1.5, -1.5};
  fingers_[0].joint_max = {1.5, 0.5, 1.5, 1.5};

  // index
  fingers_[1].name = "index";
  fingers_[1].joint_names = {"finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8"};
  fingers_[1].joint_min = {-1.5, -1.5, -1.5, -1.5};
  fingers_[1].joint_max = {0.6, 1.5, 1.5, 1.5};

  // middle      cylinder_tape : 0.4
  fingers_[2].name = "middle";
  fingers_[2].joint_names = {"finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12"};
  fingers_[2].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[2].joint_max = {0.6, 1.5, 1.5, 1.5};

  // ring        cylinder_tape : 0.4
  fingers_[3].name = "ring";
  fingers_[3].joint_names = {"finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16"};
  fingers_[3].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[3].joint_max = {0.6, 1.5, 1.5, 1.5};

  // little
  fingers_[4].name = "little";
  fingers_[4].joint_names = {"finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};
  fingers_[4].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers_[4].joint_max = {1.5, 1.5, 1.5, 1.5}; // 범위 넓혀 놓음 = urdf 랑 joint 5, 17 매칭 필요

  for (auto& finger : fingers_) {
    finger.current_joint_targets = {0.0, 0.0, 0.0, 0.0};
  }
}

// clang-format off
void TactileGraspController::init_joints() {
  hand_joint_names_ = {
      "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
      "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
      "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
      "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
      "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
  };
  init_positions_ = {    // 1.0
    0.297, -1.792, 0.0, 0.0,     // org
    // 0.12, -1.68, 0.0, 0.0,     // tennis
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0
  };
} // clang-format on

} // namespace robotis_hand_tactile