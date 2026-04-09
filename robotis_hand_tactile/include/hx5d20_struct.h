#pragma once

#include <array>
#include <string>

namespace robotis_hand_tactile
{

struct Hx5d20SensorData
{
  std::string name;
  std::array<std::string, 9> labels{};
  std::array<double, 9> values{};
};

}  // namespace robotis_hand_tactile