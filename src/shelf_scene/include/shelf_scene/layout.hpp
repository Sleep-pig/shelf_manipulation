#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace shelf_scene
{
struct Box
{
  std::string name;
  std::array<double, 3> size;
  std::array<double, 3> center;
  double yaw;
};

// Width is the clear inner width and depth is the outer depth, in metres.
std::vector<Box> layout(std::int64_t seed, double width = .36, double depth = .32);
}  // namespace shelf_scene
