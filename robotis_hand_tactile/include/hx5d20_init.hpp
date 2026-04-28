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

#pragma once

#include <string>
#include <vector>

#include "hx5d20_struct.h"

namespace robotis_hand_tactile {

/**
 * @brief Initialize HX5-D20 finger joint configuration.
 */
FingerArray init_fingers();
std::vector<std::string> init_joint_names();
std::vector<double> init_positions();

} // namespace robotis_hand_tactile
