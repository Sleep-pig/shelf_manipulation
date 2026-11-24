#pragma once

#include <chrono>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <sys/types.h>

namespace shelf_scene {

class Process {
public:
  Process(const std::vector<std::string>& command,
          const std::map<std::string, std::string>& environment = {},
          const std::filesystem::path& output = {});
  ~Process();
  Process(const Process&) = delete;
  Process& operator=(const Process&) = delete;

  std::optional<int> poll();
  void interrupt();
  void stop(std::chrono::milliseconds grace = std::chrono::seconds(8));

private:
  pid_t pid_ = -1;
  std::optional<int> status_;
  bool interrupt_sent_ = false;
};

class TemporaryDirectory {
public:
  explicit TemporaryDirectory(const std::string& prefix);
  ~TemporaryDirectory();
  TemporaryDirectory(const TemporaryDirectory&) = delete;
  TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

  const std::filesystem::path& path() const { return path_; }

private:
  std::filesystem::path path_;
};

void install_signal_handlers();
bool interrupted();

}  // namespace shelf_scene
