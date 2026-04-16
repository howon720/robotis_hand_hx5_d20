#pragma once

#include "hx5d20_struct.h"
#include <string>
#include <vector>

namespace robotis_hand_tactile {

FingerArray init_fingers();
std::vector<std::string> init_joint_names();
std::vector<double> init_positions();

} // namespace robotis_hand_tactile