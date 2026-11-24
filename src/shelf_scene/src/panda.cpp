#include "panda.hpp"

#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

namespace shelf_scene {
namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using Attributes = std::initializer_list<std::pair<const char*, std::string>>;

XMLElement* element(XMLElement* parent, const char* tag, Attributes attributes = {}) {
  auto* added = parent->InsertNewChildElement(tag);
  for (const auto& [name, value] : attributes) {
    added->SetAttribute(name, value.c_str());
  }
  return added;
}

void text_element(XMLElement* parent, const char* tag, const std::string& value) {
  element(parent, tag)->SetText(value.c_str());
}

XMLElement* read_robot(XMLDocument& document, const std::filesystem::path& path) {
  if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Cannot read robot XML " + path.string() + ": " + document.ErrorStr());
  }
  if (!document.RootElement()) {
    throw std::runtime_error("No robot element in " + path.string());
  }
  return document.RootElement();
}

std::string serialize(XMLElement* root) {
  tinyxml2::XMLPrinter printer(nullptr, true);
  root->Accept(&printer);
  return printer.CStr();
}

void add_inertials(XMLElement* robot, const std::filesystem::path& franka_share) {
  YAML::Node inertials;
  for (const auto* directory : {"robots/fer", "end_effectors/franka_hand"}) {
    const auto values = YAML::LoadFile((franka_share / directory / "inertials.yaml").string());
    for (const auto& entry : values) {
      inertials[entry.first.as<std::string>()] = entry.second;
    }
  }
  for (auto* link = robot->FirstChildElement("link"); link;
       link = link->NextSiblingElement("link")) {
    const auto* link_name = link->Attribute("name");
    if (!link_name) {
      throw std::runtime_error("Panda URDF contains an unnamed link");
    }
    std::string name = link_name;
    if (name.compare(0, 6, "panda_") == 0) {
      name.erase(0, 6);
    }
    if (name == "link8") {
      continue;  // Fixed flange frame.
    }
    const YAML::Node data = inertials[name];
    if (!data) {
      throw std::runtime_error("Missing Franka inertia for " + name);
    }
    auto* inertial = element(link, "inertial");
    auto* origin = element(inertial, "origin");
    for (const auto& entry : data["origin"]) {
      origin->SetAttribute(entry.first.Scalar().c_str(), entry.second.Scalar().c_str());
    }
    element(inertial, "mass", {{"value", data["mass"].Scalar()}});
    auto* inertia = element(inertial, "inertia");
    for (const auto& entry : data["inertia"]) {
      inertia->SetAttribute(("i" + entry.first.Scalar()).c_str(), entry.second.Scalar().c_str());
    }
  }
}

void add_finger_pads(XMLElement* robot) {
  for (const auto* finger : {"panda_leftfinger", "panda_rightfinger"}) {
    auto* link = robot->FirstChildElement("link");
    while (link && !link->Attribute("name", finger)) {
      link = link->NextSiblingElement("link");
    }
    if (!link || !link->FirstChildElement("collision")) {
      throw std::runtime_error(std::string("Missing Panda finger collision for ") + finger);
    }
    link->DeleteChild(link->FirstChildElement("collision"));
    auto* collision = element(link, "collision");
    const bool left = std::string_view(finger) == "panda_leftfinger";
    element(collision, "origin", {{"xyz", left ? "0 0.008 .029" : "0 -0.008 .029"}});
    element(element(collision, "geometry"), "box", {{"size", ".020 .016 .048"}});
    auto* surface = element(robot, "gazebo", {{"reference", finger}});
    text_element(surface, "mu1", "1.0");
    text_element(surface, "mu2", "1.0");
    text_element(surface, "minDepth", "0.0001");
  }
  element(element(robot, "gazebo"), "plugin",
          {{"name", "shelf_grasp"}, {"filename", "libshelf_grasp.so"}});
}

void add_control(XMLElement* robot, const std::filesystem::path& moveit_config,
                 const std::filesystem::path& controller_file) {
  auto* control = element(robot, "ros2_control", {{"name", "Panda"}, {"type", "system"}});
  text_element(element(control, "hardware"), "plugin", "gazebo_ros2_control/GazeboSystem");
  auto positions = YAML::LoadFile((moveit_config / "initial_positions.yaml").string())["initial_positions"];
  positions["panda_finger_joint1"] = ".04";
  positions["panda_finger_joint2"] = ".04";
  for (const auto& entry : positions) {
    const auto name = entry.first.as<std::string>();
    const bool finger = name.find("finger") != std::string::npos;
    auto* joint = element(control, "joint", {{"name", name}});
    element(joint, "command_interface", {{"name", finger ? "effort" : "position"}});
    auto* state = element(joint, "state_interface", {{"name", "position"}});
    element(state, "param", {{"name", "initial_value"}})->SetText(entry.second.Scalar().c_str());
    element(joint, "state_interface", {{"name", "velocity"}});
  }
  auto controllers = YAML::LoadFile((moveit_config / "ros2_controllers.yaml").string());
  auto manager = controllers["controller_manager"]["ros__parameters"];
  manager["update_rate"] = 500;
  const auto* hand = "panda_hand_controller";
  const auto* right_finger = "panda_right_finger_controller";
  manager[hand]["type"] = "effort_controllers/GripperActionController";
  manager[right_finger] = YAML::Clone(manager[hand]);
  auto hand_parameters = controllers[hand]["ros__parameters"];
  hand_parameters["goal_tolerance"] = .001;
  hand_parameters["allow_stalling"] = true;
  hand_parameters["stall_timeout"] = .4;
  hand_parameters["stall_velocity_threshold"] = .15;
  auto gains = hand_parameters["gains"]["panda_finger_joint1"];
  // Keep integral-valued gains as YAML doubles for the controller parameters.
  gains["p"] = "450.0";
  gains["i"] = "0.0";
  gains["d"] = "2.0";
  auto right = YAML::Clone(hand_parameters);
  right["joint"] = "panda_finger_joint2";
  right["gains"].remove("panda_finger_joint1");
  right["gains"]["panda_finger_joint2"] = YAML::Clone(gains);
  controllers[right_finger]["ros__parameters"] = right;
  write_file(controller_file, YAML::Dump(controllers));

  auto* plugin = element(element(robot, "gazebo"), "plugin",
                         {{"name", "gazebo_ros2_control"}, {"filename", "libgazebo_ros2_control.so"}});
  text_element(plugin, "parameters", controller_file.string());
}

}  // namespace

std::string robot_description(const std::filesystem::path& panda_share,
                              const std::filesystem::path& moveit_config,
                              const std::filesystem::path& franka_share,
                              const std::filesystem::path& controller_file) {
  XMLDocument document;
  auto* robot = read_robot(document, panda_share / "urdf/panda.urdf");
  add_inertials(robot, franka_share);
  add_finger_pads(robot);
  element(robot, "link", {{"name", "world"}});
  auto* mount = element(robot, "joint", {{"name", "mount"}, {"type", "fixed"}});
  element(mount, "parent", {{"link", "world"}});
  element(mount, "child", {{"link", "panda_link0"}});
  add_control(robot, moveit_config, controller_file);
  return serialize(robot);
}

std::string robot_semantic_description(const std::filesystem::path& moveit_config) {
  XMLDocument document;
  auto* robot = read_robot(document, moveit_config / "panda.srdf");
  if (auto* joint = robot->FirstChildElement("virtual_joint")) {
    robot->DeleteChild(joint);  // The URDF already mounts the Panda in world.
  }
  return serialize(robot);
}

void write_file(const std::filesystem::path& path, const std::string& contents) {
  std::ofstream output(path);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output << contents;
  output.close();
}

}  // namespace shelf_scene
