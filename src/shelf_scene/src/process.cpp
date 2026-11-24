#include "shelf_scene/process.hpp"

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <stdexcept>
#include <system_error>
#include <thread>

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace shelf_scene {
namespace {

volatile std::sig_atomic_t shutdown_requested = 0;

void request_shutdown(int) {
  shutdown_requested = 1;
}

void check(int error, const std::string& operation) {
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), operation);
  }
}

std::vector<char*> arguments(std::vector<std::string>& strings) {
  std::vector<char*> result;
  result.reserve(strings.size() + 1);
  for (auto& string : strings) {
    result.push_back(string.data());
  }
  result.push_back(nullptr);
  return result;
}

}  // namespace

Process::Process(const std::vector<std::string>& command,
                 const std::map<std::string, std::string>& environment,
                 const std::filesystem::path& output) {
  if (command.empty()) {
    throw std::invalid_argument("Empty process command");
  }
  auto command_storage = command;
  auto argv = arguments(command_storage);
  std::vector<std::string> environment_storage;
  for (char** entry = environ; *entry; ++entry) {
    const std::string value(*entry);
    if (environment.count(value.substr(0, value.find('='))) == 0) {
      environment_storage.push_back(value);
    }
  }
  for (const auto& [name, value] : environment) {
    environment_storage.push_back(name + "=" + value);
  }
  auto envp = arguments(environment_storage);

  posix_spawn_file_actions_t files;
  check(posix_spawn_file_actions_init(&files), "Initialize process redirection");
  posix_spawnattr_t attributes;
  const int init_error = posix_spawnattr_init(&attributes);
  if (init_error != 0) {
    posix_spawn_file_actions_destroy(&files);
    check(init_error, "Initialize process attributes");
  }
  try {
    if (!output.empty()) {
      check(posix_spawn_file_actions_addopen(&files, STDOUT_FILENO, output.c_str(),
                                           O_WRONLY | O_CREAT | O_TRUNC, 0666),
            "Redirect output to " + output.string());
      check(posix_spawn_file_actions_adddup2(&files, STDOUT_FILENO, STDERR_FILENO),
            "Redirect standard error");
    }
    check(posix_spawnattr_setpgroup(&attributes, 0), "Set child process group");
    check(posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP), "Set spawn flags");
    check(posix_spawnp(&pid_, argv.front(), &files, &attributes, argv.data(), envp.data()),
          "Start " + command.front());
  } catch (...) {
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&files);
    throw;
  }
  posix_spawnattr_destroy(&attributes);
  posix_spawn_file_actions_destroy(&files);
}

Process::~Process() {
  try {
    stop();
  } catch (...) {
    // The process group has already been signalled if waiting fails during cleanup.
  }
}

std::optional<int> Process::poll() {
  if (status_ || pid_ == -1) {
    return status_;
  }
  int raw_status = 0;
  pid_t result;
  do {
    result = waitpid(pid_, &raw_status, WNOHANG);
  } while (result == -1 && errno == EINTR);
  if (result == -1) {
    check(errno, "Wait for process");
  }
  if (result == pid_) {
    status_ = WIFEXITED(raw_status) ? WEXITSTATUS(raw_status) : 128 + WTERMSIG(raw_status);
  }
  return status_;
}

void Process::interrupt() {
  if (pid_ != -1 && !interrupt_sent_) {
    kill(-pid_, SIGINT);
    interrupt_sent_ = true;
  }
}

void Process::stop(std::chrono::milliseconds grace) {
  if (pid_ == -1) {
    return;
  }
  interrupt();
  const auto deadline = std::chrono::steady_clock::now() + grace;
  do {
    poll();
    if (kill(-pid_, 0) == -1 && errno == ESRCH) {
      pid_ = -1;
      return;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (true);
  kill(-pid_, SIGKILL);
  if (!status_) {
    int raw_status = 0;
    pid_t result;
    do {
      result = waitpid(pid_, &raw_status, 0);
    } while (result == -1 && errno == EINTR);
    if (result == pid_) {
      status_ = WIFEXITED(raw_status) ? WEXITSTATUS(raw_status) : 128 + WTERMSIG(raw_status);
    }
  }
  pid_ = -1;
}

TemporaryDirectory::TemporaryDirectory(const std::string& prefix) {
  auto name = (std::filesystem::temp_directory_path() / (prefix + "XXXXXX")).string();
  if (mkdtemp(name.data()) == nullptr) {
    check(errno, "Create temporary directory");
  }
  path_ = name;
}

TemporaryDirectory::~TemporaryDirectory() {
  std::error_code ignored;
  std::filesystem::remove_all(path_, ignored);
}

void install_signal_handlers() {
  struct sigaction action {};
  action.sa_handler = request_shutdown;
  sigemptyset(&action.sa_mask);
  if (sigaction(SIGINT, &action, nullptr) == -1 || sigaction(SIGTERM, &action, nullptr) == -1) {
    check(errno, "Install shutdown signal handler");
  }
}

bool interrupted() {
  return shutdown_requested != 0;
}

}  // namespace shelf_scene
