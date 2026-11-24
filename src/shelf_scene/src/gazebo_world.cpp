#include "shelf_scene/world.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  std::int64_t seed = 0;
  double width = .36;
  double depth = .32;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--help" || argument == "-h") {
        std::cout << "Usage: gazebo_world [--seed SEED] [--width WIDTH] [--depth DEPTH]\n"
                     "Generate a Gazebo Classic world from the shelf geometry.\n";
        return 0;
      }

      const auto equals = argument.find('=');
      const std::string option = argument.substr(0, equals);
      if (option != "--seed" && option != "--width" && option != "--depth") {
        throw std::invalid_argument("Unknown argument " + argument);
      }
      if (equals == std::string::npos && i + 1 == argc) {
        throw std::invalid_argument("Missing value for " + option);
      }
      const std::string value = equals == std::string::npos ? argv[++i] : argument.substr(equals + 1);
      std::size_t end = 0;
      try {
        if (option == "--seed") {
          seed = std::stoll(value, &end);
        } else if (option == "--width") {
          width = std::stod(value, &end);
        } else {
          depth = std::stod(value, &end);
        }
        if (end != value.size()) {
          throw std::invalid_argument("Trailing characters");
        }
      } catch (const std::logic_error &) {
        throw std::invalid_argument("Invalid value for " + option + ": " + value);
      }
    }
    std::cout << shelf_scene::world(seed, width, depth);
  } catch (const std::exception & error) {
    std::cerr << "gazebo_world: " << error.what() << '\n';
    return 2;
  }
  return 0;
}
