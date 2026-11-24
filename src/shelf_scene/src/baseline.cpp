#include "panda.hpp"
#include "shelf_scene/process.hpp"
#include "shelf_scene/world.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace {

namespace fs = std::filesystem;
using shelf_scene::Process;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

struct Options {
  std::int64_t seed = 7;
  double width = .36;
  double depth = .32;
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; i += 2) {
    const std::string name = argv[i];
    if (i + 1 >= argc) {
      throw std::invalid_argument("Missing value for " + name);
    }
    const std::string value = argv[i + 1];
    std::size_t end = 0;
    if (name == "--seed") {
      options.seed = std::stoll(value, &end);
    } else if (name == "--width") {
      options.width = std::stod(value, &end);
    } else if (name == "--depth") {
      options.depth = std::stod(value, &end);
    } else {
      throw std::invalid_argument("Unknown option " + name);
    }
    if (end != value.size()) {
      throw std::invalid_argument("Invalid value for " + name + ": " + value);
    }
  }
  return options;
}

std::string executable(const std::string& package, const std::string& name) {
  return (fs::path(ament_index_cpp::get_package_prefix(package)) / "lib" / package / name).string();
}

std::string prepend_path(const fs::path& path, const char* variable) {
  const char* previous = std::getenv(variable);
  return previous && *previous ? path.string() + ':' + previous : path.string();
}

void write_parameters(const fs::path& path, const YAML::Node& parameters) {
  YAML::Node root;
  root["/**"]["ros__parameters"] = parameters;
  shelf_scene::write_file(path, YAML::Dump(root));
}

struct Child {
  std::string name;
  std::unique_ptr<Process> process;
  bool required;
};

void check_running(const std::vector<Child>& children) {
  for (const auto& child : children) {
    if (child.required) {
      if (const auto status = child.process->poll()) {
        throw std::runtime_error(child.name + " exited with status " + std::to_string(*status));
      }
    }
  }
}

void stop_children(const std::vector<Child>& children) {
  for (const auto& child : children) {
    child.process->interrupt();
  }
  const auto deadline = Clock::now() + 3s;
  for (auto child = children.rbegin(); child != children.rend(); ++child) {
    child->process->stop(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()));
  }
}

void spawn_panda(const std::string& description, const std::vector<Child>& children) {
  auto node = std::make_shared<rclcpp::Node>("shelf_baseline");
  auto spawn = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
  auto parameters = node->create_client<rcl_interfaces::srv::GetParameters>(
      "/robot_state_publisher/get_parameters");
  const auto deadline = Clock::now() + 30s;
  // gazebo_ros2_control reads the robot description from robot_state_publisher.
  while (!spawn->wait_for_service(100ms) || !parameters->wait_for_service(100ms)) {
    if (shelf_scene::interrupted()) {
      return;
    }
    check_running(children);
    if (Clock::now() >= deadline) {
      throw std::runtime_error("Timed out waiting for Gazebo or robot_state_publisher");
    }
  }
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = "panda";
  request->xml = description;
  request->reference_frame = "world";
  request->initial_pose.orientation.w = 1.;
  auto future = spawn->async_send_request(request);
  const auto spawn_deadline = Clock::now() + 30s;
  while (!shelf_scene::interrupted()) {
    if (rclcpp::spin_until_future_complete(node, future, 100ms) == rclcpp::FutureReturnCode::SUCCESS) {
      const auto result = future.get();
      if (!result->success) {
        throw std::runtime_error("Gazebo could not spawn Panda: " + result->status_message);
      }
      return;
    }
    check_running(children);
    if (Clock::now() >= spawn_deadline) {
      throw std::runtime_error("SpawnEntity timed out");
    }
  }
}

void run(const Options& options) {
  const auto sdf = shelf_scene::world(options.seed, options.width, options.depth);
  const fs::path panda = ament_index_cpp::get_package_share_directory("moveit_resources_panda_description");
  const auto config = fs::path(ament_index_cpp::get_package_share_directory(
      "moveit_resources_panda_moveit_config")) / "config";
  const fs::path franka = ament_index_cpp::get_package_share_directory("franka_description");
  const fs::path grasp_plugin = ament_index_cpp::get_package_prefix("shelf_grasp_plugin");
  shelf_scene::TemporaryDirectory directory("shelf-world-");
  const auto world_file = directory.path() / "scene.world";
  const auto planning_file = directory.path() / "planning.yaml";
  const auto move_group_file = directory.path() / "move_group.yaml";
  const auto publisher_file = directory.path() / "publisher.yaml";
  const auto robot_file = directory.path() / "robot_state_publisher.yaml";
  shelf_scene::write_file(world_file, sdf);
  const auto description = shelf_scene::robot_description(
      panda, config, franka, directory.path() / "controllers.yaml");

  YAML::Node robot;
  robot["use_sim_time"] = true;
  robot["robot_description"] = description;
  write_parameters(robot_file, robot);
  auto planning = YAML::Clone(robot);
  planning["robot_description_semantic"] = shelf_scene::robot_semantic_description(config);
  planning["robot_description_kinematics"] = YAML::LoadFile((config / "kinematics.yaml").string());
  planning["robot_description_planning"] = YAML::LoadFile((config / "joint_limits.yaml").string());
  planning["planning_pipelines"].push_back("ompl");
  planning["default_planning_pipeline"] = "ompl";
  auto ompl = YAML::LoadFile((config / "ompl_planning.yaml").string());
  ompl["panda_arm"]["longest_valid_segment_fraction"] = .001;
  planning["ompl"] = ompl;
  write_parameters(planning_file, planning);
  write_parameters(move_group_file, YAML::LoadFile((config / "moveit_controllers.yaml").string()));

  YAML::Node scene;
  scene["seed"] = options.seed;
  scene["width"] = options.width;
  scene["depth"] = options.depth;
  scene["apply_to_move_group"] = true;
  write_parameters(publisher_file, scene);

  std::vector<Child> children;
  const auto start = [&](const std::vector<std::string>& command,
                         const std::map<std::string, std::string>& environment = {},
                         bool required = true) -> Process& {
    children.push_back({command.front(), std::make_unique<Process>(command, environment), required});
    return *children.back().process;
  };
  try {
    start({"gzserver", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so",
           world_file.string()},
          {{"GAZEBO_MODEL_PATH", prepend_path(panda.parent_path(), "GAZEBO_MODEL_PATH")},
           {"GAZEBO_PLUGIN_PATH", prepend_path(grasp_plugin / "lib", "GAZEBO_PLUGIN_PATH")}});
    start({executable("moveit_ros_move_group", "move_group"), "--ros-args",
           "--params-file", planning_file.string(), "--params-file", move_group_file.string()});
    start({executable("shelf_planning", "evaluate"), "--ros-args", "--params-file", planning_file.string()});
    start({executable("robot_state_publisher", "robot_state_publisher"), "--ros-args",
           "--params-file", robot_file.string()});
    start({executable("shelf_scene", "publish_scene"), "--ros-args", "--params-file", publisher_file.string()});
    spawn_panda(description, children);
    if (!shelf_scene::interrupted()) {
      auto& spawner = start({executable("controller_manager", "spawner"), "joint_state_broadcaster",
                             "panda_arm_controller", "panda_hand_controller",
                             "panda_right_finger_controller", "--controller-manager-timeout", "60"},
                            {}, false);
      bool ready = false;
      while (!shelf_scene::interrupted()) {
        check_running(children);
        if (!ready) {
          if (const auto status = spawner.poll()) {
            if (*status != 0) {
              throw std::runtime_error("Controller spawner exited with status " + std::to_string(*status));
            }
            ready = true;
            std::cout << "Shelf controllers active" << std::endl;
          }
        }
        std::this_thread::sleep_for(100ms);
      }
    }
  } catch (...) {
    stop_children(children);
    throw;
  }
  stop_children(children);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_options(argc, argv);
    shelf_scene::install_signal_handlers();
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    run(options);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Baseline: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
