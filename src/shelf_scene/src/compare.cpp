#include "shelf_scene/process.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <system_error>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;
using Clock = std::chrono::steady_clock;
using shelf_scene::Process;
using namespace std::chrono_literals;

const std::vector<std::string> strategies{"fixed", "multi", "precheck"};

struct Options
{
  fs::path output;
  std::vector<std::int64_t> seeds{7, 8};
  std::vector<double> widths{.30, .36};
  std::vector<double> depths{.28, .32};
  double timeout = 120.;
  std::int64_t domain = 95;
  bool help = false;
};

const char usage[] =
  "Usage: ros2 run shelf_scene compare OUTPUT [--seeds SEED ...] [--widths WIDTH ...]\n"
  "       [--depths DEPTH ...] [--timeout SECONDS] [--domain ID]\n"
  "Run fixed, multi, and precheck in fresh Gazebo worlds and save logs and ROS bags.\n"
  "Defaults: seeds 7 8, widths 0.30 0.36, depths 0.28 0.32, timeout 120, domain 95.\n";

std::int64_t integer(const std::string & text, const std::string & option)
{
  std::size_t length = 0;
  try {
    const auto value = std::stoll(text, &length);
    if (length == text.size()) {
      return value;
    }
  } catch (const std::logic_error &) {
  }
  throw std::invalid_argument(option + " expects an integer, got " + text);
}

double number(const std::string & text, const std::string & option)
{
  std::size_t length = 0;
  try {
    const double value = std::stod(text, &length);
    if (length == text.size() && std::isfinite(value)) {
      return value;
    }
  } catch (const std::logic_error &) {
  }
  throw std::invalid_argument(option + " expects a finite number, got " + text);
}

std::vector<std::string> arguments(
  int & index, int argc, char ** argv, const std::string & option,
  const std::string & first_value, bool has_equals, bool multiple)
{
  std::vector<std::string> values;
  if (has_equals) {
    values.push_back(first_value);
  } else {
    while (index + 1 < argc && (multiple || values.empty())) {
      const std::string next = argv[index + 1];
      if (next.rfind("--", 0) == 0 || next == "-h") {
        break;
      }
      values.push_back(next);
      ++index;
    }
  }
  if (values.empty()) {
    throw std::invalid_argument(option + " requires a value");
  }
  return values;
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  bool positional_only = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (positional_only || argument.empty() || argument.front() != '-') {
      if (!options.output.empty()) {
        throw std::invalid_argument("expected one output directory");
      }
      options.output = argument;
      continue;
    }
    if (argument == "--") {
      positional_only = true;
      continue;
    }
    if (argument == "--help" || argument == "-h") {
      options.help = true;
      return options;
    }
    const auto equals = argument.find('=');
    const auto option = argument.substr(0, equals);
    const bool has_equals = equals != std::string::npos;
    const auto first_value = has_equals ? argument.substr(equals + 1) : std::string{};
    if (option == "--timeout" || option == "--domain") {
      const auto values = arguments(index, argc, argv, option, first_value, has_equals, false);
      if (option == "--timeout") {
        options.timeout = number(values.front(), option);
      } else {
        options.domain = integer(values.front(), option);
      }
    } else if (option == "--seeds") {
      const auto values = arguments(index, argc, argv, option, first_value, has_equals, true);
      options.seeds.clear();
      for (const auto & value : values) {
        options.seeds.push_back(integer(value, option));
      }
    } else if (option == "--widths" || option == "--depths") {
      const auto values = arguments(index, argc, argv, option, first_value, has_equals, true);
      auto & dimensions = option == "--widths" ? options.widths : options.depths;
      dimensions.clear();
      for (const auto & value : values) {
        dimensions.push_back(number(value, option));
      }
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.output.empty()) {
    throw std::invalid_argument("an output directory is required");
  }
  return options;
}

int gazebo_port()
{
  const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd == -1) {
    throw std::system_error(errno, std::generic_category(), "creating Gazebo port socket");
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  auto * socket_address = reinterpret_cast<sockaddr *>(&address);
  socklen_t length = sizeof(address);
  if (bind(fd, socket_address, length) == -1 || getsockname(fd, socket_address, &length) == -1) {
    const int error = errno;
    close(fd);
    throw std::system_error(error, std::generic_category(), "selecting Gazebo port");
  }
  close(fd);
  return ntohs(address.sin_port);
}

void write_file(const fs::path & path, const std::string & text,
  std::ios::openmode mode = std::ios::trunc)
{
  std::ofstream output;
  output.exceptions(std::ios::badbit | std::ios::failbit);
  output.open(path, mode);
  output << text;
  output.close();
}

struct Interrupted {};

void check_interruption()
{
  if (shelf_scene::interrupted()) {
    throw Interrupted{};
  }
}

void wait_for_baseline(Process & launch, const fs::path & log_path)
{
  const auto deadline = Clock::now() + 45s;
  while (Clock::now() < deadline) {
    check_interruption();
    if (launch.poll()) {
      break;
    }
    std::ifstream input(log_path);
    const std::string log{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    if (log.find("Shelf geometry applied to MoveIt") != std::string::npos &&
      log.find("Shelf controllers active") != std::string::npos)
    {
      return;
    }
    std::this_thread::sleep_for(200ms);
  }
  throw std::runtime_error("baseline launch failed to become ready");
}

Json runner_result(const fs::path & log_path)
{
  std::ifstream execution(log_path);
  std::string payload;
  const std::string prefix = "TRIAL_RESULT ";
  for (std::string line; std::getline(execution, line);) {
    if (line.rfind(prefix, 0) == 0) {
      payload = line.substr(prefix.size());
    }
  }
  if (payload.empty()) {
    throw std::runtime_error("grasp runner exited without a TRIAL_RESULT");
  }
  auto result = Json::parse(payload, nullptr, false);
  if (!result.is_object() || !result.contains("success") || !result["success"].is_boolean() ||
    !result.contains("grasped") || !result["grasped"].is_boolean() ||
    !result.contains("stage") || !result["stage"].is_string() ||
    !result.contains("planning_s") ||
    !(result["planning_s"].is_null() || result["planning_s"].is_number()))
  {
    throw std::runtime_error("grasp runner returned an invalid TRIAL_RESULT");
  }
  return result;
}

struct TrialResult
{
  Json row;
  bool valid;
};

TrialResult run_trial(const fs::path & directory, std::int64_t seed, double width, double depth,
  const std::string & strategy, double timeout, const std::map<std::string, std::string> & env)
{
  if (!fs::create_directory(directory)) {
    throw std::runtime_error("trial directory already exists: " + directory.string());
  }
  const auto launch_log = directory / "launch.log";
  const auto record_log = directory / "record.log";
  const auto execution_log = directory / "execution.log";
  for (const auto & log : {launch_log, record_log, execution_log}) {
    write_file(log, "");
  }
  Json row{{"success", false}, {"stage", "startup"}, {"grasped", false}, {"planning_s", nullptr}};
  bool valid = false;
  try {
    bool timed_out = false;
    int runner_exit = 0;
    {
      Process launch({"ros2", "launch", "shelf_scene", "baseline.launch.xml",
          "seed:=" + std::to_string(seed), "width:=" + Json(width).dump(),
          "depth:=" + Json(depth).dump()}, env, launch_log);
      wait_for_baseline(launch, launch_log);
      Process record({"ros2", "bag", "record", "-o", (directory / "replay").string(),
          "/clock", "/joint_states", "/tf", "/tf_static", "/gazebo/model_states",
          "/monitored_planning_scene", "/shelf/markers"}, env, record_log);
      Process runner({"ros2", "run", "shelf_scene", "fixed_grasp", "--ros-args", "-p",
          "strategy:=" + strategy}, env, execution_log);
      row["stage"] = "execution";
      const auto deadline = Clock::now() + std::chrono::duration<double>(timeout);
      for (;;) {
        check_interruption();
        if (launch.poll()) {
          throw std::runtime_error("baseline launch exited before the trial ended");
        }
        if (record.poll()) {
          throw std::runtime_error("rosbag recorder exited before the trial ended");
        }
        if (const auto status = runner.poll()) {
          runner_exit = *status;
          break;
        }
        if (Clock::now() >= deadline) {
          timed_out = true;
          break;
        }
        std::this_thread::sleep_for(50ms);
      }
    }
    check_interruption();
    if (timed_out) {
      row["stage"] = "timeout";
      row["error"] = "trial deadline exceeded";
    } else {
      row = runner_result(execution_log);
      if (row.at("success").get<bool>() != (runner_exit == 0)) {
        throw std::runtime_error("grasp runner exit status disagrees with TRIAL_RESULT");
      }
    }
    const auto metadata = directory / "replay" / "metadata.yaml";
    if (!fs::is_regular_file(metadata) || fs::file_size(metadata) == 0) {
      throw std::runtime_error("rosbag recorder did not save metadata.yaml");
    }
    valid = true;
  } catch (const std::runtime_error & error) {
    row["error"] = error.what();
  }
  row["seed"] = seed;
  row["width"] = width;
  row["depth"] = depth;
  row["strategy"] = strategy;
  return {std::move(row), valid};
}

struct Group
{
  double width;
  double depth;
  std::string strategy;
  std::size_t trials = 0;
  std::size_t successes = 0;
  std::size_t verified = 0;
  std::size_t failed_withdrawals = 0;
  std::vector<double> times{};

  void add(const Json & row)
  {
    ++trials;
    successes += row.at("success").get<bool>();
    if (row.at("grasped").get<bool>()) {
      ++verified;
      failed_withdrawals += row.at("stage") == "withdraw";
    }
    if (!row.at("planning_s").is_null()) {
      const auto time = row.at("planning_s").get<double>();
      times.insert(std::lower_bound(times.begin(), times.end(), time), time);
    }
  }

  Json summary() const
  {
    const auto rank = static_cast<std::size_t>(std::ceil(.95 * times.size()));
    const auto withdraw_rate = verified ?
      Json(static_cast<double>(failed_withdrawals) / verified) : Json(nullptr);
    return {{"width", width}, {"depth", depth}, {"strategy", strategy}, {"trials", trials},
      {"success_rate", static_cast<double>(successes) / trials}, {"verified_grasps", verified},
      {"withdraw_failure_rate", withdraw_rate},
      {"planning_p95_s", times.empty() ? Json(nullptr) : Json(times[rank - 1])},
      {"timed_trials", times.size()}};
  }
};

int compare(const Options & options)
{
  const auto output = fs::weakly_canonical(fs::absolute(options.output));
  if (!fs::create_directories(output)) {
    throw std::runtime_error("output directory already exists: " + output.string());
  }
  const Json config{{"output", output.string()}, {"seeds", options.seeds},
    {"widths", options.widths}, {"depths", options.depths}, {"timeout", options.timeout},
    {"domain", options.domain}, {"strategies", strategies}};
  write_file(output / "config.json", config.dump(2));

  shelf_scene::TemporaryDirectory logs("shelf-study-logs-");
  std::map<std::tuple<double, double, std::string>, std::size_t> group_indices;
  std::vector<Group> groups;
  bool complete = true;
  for (const auto width : options.widths) {
    for (const auto depth : options.depths) {
      for (const auto seed : options.seeds) {
        for (const auto & strategy : strategies) {
          check_interruption();
          const std::map<std::string, std::string> env{
            {"ROS_DOMAIN_ID", std::to_string(options.domain)}, {"ROS_LOG_DIR", logs.path().string()},
            {"GAZEBO_MASTER_URI", "http://localhost:" + std::to_string(gazebo_port())},
            {"GAZEBO_MODEL_DATABASE_URI", ""}};
          const auto name = "w" + Json(width).dump() + "-d" + Json(depth).dump() +
            "-s" + std::to_string(seed) + "-" + strategy;
          const auto result = run_trial(output / name, seed, width, depth, strategy, options.timeout, env);
          const auto & row = result.row;
          write_file(output / "trials.jsonl", row.dump() + '\n', std::ios::app);
          const auto [entry, inserted] = group_indices.try_emplace(
            std::make_tuple(width, depth, strategy), groups.size());
          if (inserted) {
            groups.push_back(Group{width, depth, strategy});
          }
          groups[entry->second].add(row);
          auto summary = Json::array();
          for (const auto & group : groups) {
            summary.push_back(group.summary());
          }
          write_file(output / "summary.json", summary.dump(2));
          std::cout << name << ' ' << row.at("stage").get<std::string>() << std::endl;
          if (!result.valid) {
            complete = false;
            std::cerr << name << ": " << row.at("error").get<std::string>() << '\n';
          }
        }
      }
    }
  }
  check_interruption();
  return complete ? EXIT_SUCCESS : EXIT_FAILURE;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    if (options.help) {
      std::cout << usage;
      return EXIT_SUCCESS;
    }
    shelf_scene::install_signal_handlers();
    return compare(options);
  } catch (const Interrupted &) {
    std::cerr << "Shelf comparison interrupted\n";
    return 130;
  } catch (const std::invalid_argument & error) {
    std::cerr << "compare: " << error.what() << '\n' << usage;
  } catch (const std::exception & error) {
    std::cerr << "compare: " << error.what() << '\n';
  }
  return EXIT_FAILURE;
}
