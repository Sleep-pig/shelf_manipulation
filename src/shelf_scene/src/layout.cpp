#include "shelf_scene/layout.hpp"

#include <algorithm>
#include <random>
#include <stdexcept>

namespace shelf_scene
{
namespace
{
// Match the MT19937 initialization used by Python's random.Random(int).
// Integer seeds are split into little-endian 32-bit words after taking their magnitude.
class PythonSeed
{
public:
  using result_type = std::uint32_t;

  explicit PythonSeed(std::int64_t seed)
  {
    auto magnitude = static_cast<std::uint64_t>(seed);
    if (seed < 0) {
      magnitude = std::uint64_t{0} - magnitude;
    }
    words_[0] = static_cast<result_type>(magnitude);
    words_[1] = static_cast<result_type>(magnitude >> 32);
    word_count_ = words_[1] == 0 ? 1 : 2;
  }

  template<typename Iterator>
  void generate(Iterator first, Iterator last)
  {
    State state;
    state[0] = 19650218u;
    for (std::size_t i = 1; i < state.size(); ++i) {
      state[i] = 1812433253u * (state[i - 1] ^ (state[i - 1] >> 30)) +
        static_cast<result_type>(i);
    }

    std::size_t i = 1;
    std::size_t word = 0;
    for (std::size_t count = 0; count < state.size(); ++count) {
      state[i] = (state[i] ^ ((state[i - 1] ^ (state[i - 1] >> 30)) * 1664525u)) +
        words_[word] + static_cast<result_type>(word);
      advance(state, i);
      word = (word + 1) % word_count_;
    }
    for (std::size_t count = 1; count < state.size(); ++count) {
      state[i] = (state[i] ^ ((state[i - 1] ^ (state[i - 1] >> 30)) * 1566083941u)) -
        static_cast<result_type>(i);
      advance(state, i);
    }
    state[0] = 0x80000000u;
    std::copy_n(state.begin(), last - first, first);
  }

private:
  using State = std::array<result_type, std::mt19937::state_size>;

  static void advance(State & state, std::size_t & i)
  {
    if (++i == state.size()) {
      state[0] = state.back();
      i = 1;
    }
  }

  std::array<result_type, 2> words_;
  std::size_t word_count_;
};

double uniform(std::mt19937 & random, double low, double high)
{
  // Python constructs a 53-bit fraction from two consecutive engine outputs.
  const std::uint64_t high_bits = random() >> 5;
  const std::uint64_t low_bits = random() >> 6;
  constexpr auto scale = std::uint64_t{1} << 53;
  const double unit = static_cast<double>((high_bits << 26) | low_bits) / scale;
  // Round the product separately, as Python does, even when the compiler supports FMA.
  const volatile double offset = (high - low) * unit;
  return low + offset;
}
}  // namespace

std::vector<Box> layout(std::int64_t seed, double width, double depth)
{
  if (!(.24 <= width && width <= .50 && .23 <= depth && depth <= .40)) {
    throw std::invalid_argument("Supported shelf widths: 0.24–0.50 m; depths: 0.23–0.40 m");
  }

  constexpr double front = .37;
  const double center_x = front + depth / 2;
  const double side_y = width / 2 + .01;
  PythonSeed seed_sequence(seed);
  std::mt19937 random(seed_sequence);
  const double target_x = uniform(random, .44, front + depth - .10);
  const double target_y = uniform(
    random, std::max(-.07, -.5 * width + .045), std::min(.03, .5 * width - .12));
  const double target_yaw = uniform(random, -.6, .6);

  return {
    {"floor", {depth, width + .04, .02}, {center_x, 0., .20}, 0.},
    {"ceiling", {depth, width + .04, .02}, {center_x, 0., .48}, 0.},
    {"left", {depth, .02, .26}, {center_x, side_y, .34}, 0.},
    {"right", {depth, .02, .26}, {center_x, -side_y, .34}, 0.},
    {"back", {.02, width, .26}, {front + depth - .01, 0., .34}, 0.},
    {"target", {.07, .04, .06}, {target_x, target_y, .24}, target_yaw},
    {"obstacle", {.08, .05, .12}, {.49, width / 2 - .05, .27}, 0.},
    {"tray", {.20, .18, .02}, {.32, -.34, .15}, 0.},
  };
}
}  // namespace shelf_scene
