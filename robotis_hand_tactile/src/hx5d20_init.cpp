// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Howon Kim

#include "hx5d20_init.hpp"

namespace robotis_hand_tactile {

FingerArray init_fingers() {
  FingerArray fingers{};

  // Thumb
  fingers[0].name = "thumb";
  fingers[0].joint_names = {"finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4"};
  fingers[0].joint_min = {-1.5, -3.5, -1.5, -1.5};
  fingers[0].joint_max = {1.5, 0.5, 1.5, 1.5};

  // Index finger
  fingers[1].name = "index";
  fingers[1].joint_names = {"finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8"};
  fingers[1].joint_min = {-1.5, -1.5, -1.5, -1.5};
  fingers[1].joint_max = {0.6, 1.5, 1.5, 1.5};

  // Middle finger
  fingers[2].name = "middle";
  fingers[2].joint_names = {"finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12"};
  fingers[2].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers[2].joint_max = {0.6, 1.5, 1.5, 1.5};

  // Ring finger
  fingers[3].name = "ring";
  fingers[3].joint_names = {"finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16"};
  fingers[3].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers[3].joint_max = {0.6, 1.5, 1.5, 1.5};

  // Little finger
  fingers[4].name = "little";
  fingers[4].joint_names = {"finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"};
  fingers[4].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers[4].joint_max = {1.5, 1.5, 1.5, 1.5}; // 범위 넓혀 놓음 = urdf 랑 joint 5, 17 매칭 필요

  for (auto& finger : fingers) {
    finger.current_joint_targets = {0.0, 0.0, 0.0, 0.0};
  }

  return fingers;
}

// clang-format off
std::vector<std::string> init_joint_names() {
  return {
      "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
      "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
      "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
      "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
      "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
  };
}

std::vector<double> init_positions() {
  return {   // 1.0
    0.297, -1.792, 0.0, 0.0,     // org
    // 0.12, -1.68, 0.0, 0.0,     // tennis 3pinch
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0,
    0.0,    0.8,   0.0, 0.0
  };

  // // 작은 과일 들 , maxim
  // std::vector<double> init_positions() {
  // return {   // 1.0
  //   0.297, -1.792, 0.3, - 0.2,
  //   0.0,    1.2,   0.0, 0.0,
  //   0.0,    1.2,   0.0, 0.0,
  //   0.0,    1.2,   0.0, 0.0,
  //   0.0,    1.2,   0.0, 0.0
  // };

  // //hold_pinch
  // std::vector<double> init_positions() {
  // return {   // 1.0
  //   // 0.0, -1.57, 0.0, 0.0,     // org
  //   0.12, -1.68, 0.0, 0.0,    // 3pinch
  //   0.0,    1.0,   0.0, 0.0,
  //   0.0,    1.0,   0.0, 0.0,
  //   0.0,    1.5,   0.0, 0.0,
  //   0.0,    1.5,   0.0, 0.0
  // };


} // clang-format on

} // namespace robotis_hand_tactile