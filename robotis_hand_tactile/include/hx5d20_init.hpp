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
