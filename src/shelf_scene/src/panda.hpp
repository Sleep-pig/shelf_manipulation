#pragma once

#include <filesystem>
#include <string>

namespace shelf_scene {

std::string robot_description(const std::filesystem::path& panda_share,
                              const std::filesystem::path& moveit_config,
                              const std::filesystem::path& franka_share,
                              const std::filesystem::path& controller_file);

std::string robot_semantic_description(const std::filesystem::path& moveit_config);
void write_file(const std::filesystem::path& path, const std::string& contents);

}  // namespace shelf_scene
