#include "shelf_scene/layout.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <gazebo_msgs/msg/entity_state.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shelf_planning/srv/evaluate_candidate.hpp>

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using control_msgs::action::GripperCommand;
using gazebo_msgs::srv::GetEntityState;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using moveit_msgs::action::ExecuteTrajectory;
using moveit_msgs::action::MoveGroup;
using moveit_msgs::msg::CollisionObject;
using moveit_msgs::msg::MoveItErrorCodes;
using moveit_msgs::msg::RobotState;
using moveit_msgs::msg::RobotTrajectory;
using moveit_msgs::srv::ApplyPlanningScene;
using moveit_msgs::srv::GetCartesianPath;
using moveit_msgs::srv::GetPlanningScene;
using moveit_msgs::srv::GetPositionIK;
using sensor_msgs::msg::JointState;
using shape_msgs::msg::SolidPrimitive;
using shelf_planning::srv::EvaluateCandidate;

constexpr auto kGroup = "panda_arm";
constexpr auto kHand = "panda_hand";
constexpr auto kTarget = "target";
constexpr auto kWorld = "world";
constexpr double kPregraspSetback = .22;
constexpr double kGraspSetback = .108;
constexpr double kLiftHeight = .035;
constexpr double kRetreatX = .26;
constexpr double kOpenWidth = .08;

class MissedGrasp : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class RequestTimeout : public std::exception {
 public:
  const char* what() const noexcept override { return "ROS request timed out"; }
};

struct Candidate {
  double clearance;
  double travel;
  double yaw;
  RobotState start;
  RobotTrajectory path;
  std::string label;
};

bool is_arm_joint(const std::string& name) {
  return name.rfind("panda_joint", 0) == 0;
}

const std::array<double, 3>& dimensions(const std::string& name) {
  static const auto boxes = shelf_scene::layout(0);
  const auto box = std::find_if(boxes.begin(), boxes.end(), [&name](const auto& entry) {
    return entry.name == name;
  });
  if (box == boxes.end()) {
    throw std::logic_error("Missing geometry for " + name);
  }
  return box->size;
}

class FixedGrasp : public rclcpp::Node {
 public:
  FixedGrasp() : Node("fixed_grasp"), strategy_(declare_parameter<std::string>("strategy", "fixed")) {
    if (strategy_ != "fixed" && strategy_ != "multi" && strategy_ != "precheck") {
      throw std::invalid_argument("strategy must be fixed, multi or precheck");
    }
  }

  void run() {
    for (int attempt = 1; attempt <= 2; ++attempt) {
      attempts_ = attempt;
      try {
        pick_place();
        return;
      } catch (const MissedGrasp& error) {
        if (attempt == 2) {
          throw;
        }
        RCLCPP_INFO(get_logger(), "%s; reobserving target for final attempt", error.what());
      }
    }
  }

  nlohmann::json result(const std::optional<std::string>& error) const {
    return {{"strategy", strategy_}, {"success", stage_ == "complete"}, {"stage", stage_},
            {"grasped", grasped_}, {"attempts", attempts_}, {"planning_s", planning_seconds_},
            {"rejections", rejections_}, {"error", error ? nlohmann::json(*error) : nlohmann::json()}};
  }

 private:
  template <typename Future>
  auto wait(Future future) {
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 40s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw RequestTimeout();
    }
    return future.get();
  }

  template <typename Service>
  typename Service::Response::SharedPtr call_service(
      const std::string& name, const typename Service::Request& request) {
    auto client = create_client<Service>(name);
    if (!client->wait_for_service(10s)) {
      throw std::runtime_error(name + " unavailable");
    }
    const auto started = Clock::now();
    auto reply = wait(client->async_send_request(std::make_shared<typename Service::Request>(request)));
    if constexpr (std::is_same_v<Service, GetCartesianPath> ||
                  std::is_same_v<Service, GetPositionIK> ||
                  std::is_same_v<Service, EvaluateCandidate>) {
      planning_seconds_ += std::chrono::duration<double>(Clock::now() - started).count();
    }
    return reply;
  }

  template <typename Action>
  std::shared_ptr<const typename Action::Result> call_action(
      const std::string& name, const typename Action::Goal& goal) {
    auto client = rclcpp_action::create_client<Action>(this, name);
    if (!client->wait_for_action_server(10s)) {
      throw std::runtime_error(name + " unavailable");
    }
    const auto handle = wait(client->async_send_goal(goal));
    if (!handle) {
      throw std::runtime_error(name + " rejected");
    }
    const auto result = wait(client->async_get_result(handle));
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      std::ostringstream message;
      message << name << " failed: status=" << static_cast<int>(result.code);
      if (result.result) {
        message << ", error=" << result.result->error_code.val;
      }
      throw std::runtime_error(message.str());
    }
    return result.result;
  }

  Pose hand_pose(const Point& target, double setback) const {
    Pose pose;
    pose.position.x = target.x - setback * std::cos(yaw_);
    pose.position.y = target.y - setback * std::sin(yaw_);
    pose.position.z = target.z + .015;
    // Rz(yaw) Ry(pi/2) points the fingers along the horizontal approach.
    pose.orientation.y = pose.orientation.w = std::sqrt(.5) * std::cos(yaw_ / 2.);
    pose.orientation.z = std::sqrt(.5) * std::sin(yaw_ / 2.);
    pose.orientation.x = -pose.orientation.z;
    return pose;
  }

  Pose lift_pose(const Point& target) const {
    auto lift = hand_pose(target, kGraspSetback);
    lift.position.z += kLiftHeight;
    return lift;
  }

  static Pose retreat_pose(Pose lift) {
    lift.position.x = kRetreatX;
    return lift;
  }

  RobotState ik(const Pose& pose, const RobotState* seed = nullptr) {
    GetPositionIK::Request request;
    auto& ik = request.ik_request;
    ik.group_name = kGroup;
    ik.ik_link_name = kHand;
    ik.pose_stamped.header.frame_id = kWorld;
    ik.pose_stamped.pose = pose;
    if (seed) {
      ik.robot_state = *seed;
    }
    ik.robot_state.is_diff = true;
    ik.avoid_collisions = true;
    ik.timeout.sec = 2;
    const auto answer = call_service<GetPositionIK>("compute_ik", request);
    if (answer->error_code.val != MoveItErrorCodes::SUCCESS) {
      throw std::runtime_error("Pregrasp IK failed: " + std::to_string(answer->error_code.val));
    }
    return std::move(answer->solution);
  }

  void pregrasp(const Pose& pose) {
    move_joints(ik(pose).joint_state);
  }

  void move_joints(const JointState& joints) {
    MoveGroup::Goal goal;
    auto& request = goal.request;
    request.group_name = kGroup;
    request.allowed_planning_time = 5.;
    request.num_planning_attempts = 3;
    request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = .35;
    auto& constraints = request.goal_constraints.emplace_back().joint_constraints;
    const auto joint_count = std::min(joints.name.size(), joints.position.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
      if (is_arm_joint(joints.name[i])) {
        auto& constraint = constraints.emplace_back();
        constraint.joint_name = joints.name[i];
        constraint.position = joints.position[i];
        constraint.tolerance_above = constraint.tolerance_below = .002;
        constraint.weight = 1.;
      }
    }
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.plan_only = true;

    const auto started = Clock::now();
    std::shared_ptr<const MoveGroup::Result> result;
    try {
      result = call_action<MoveGroup>("move_action", goal);
    } catch (...) {
      planning_seconds_ += std::chrono::duration<double>(Clock::now() - started).count();
      throw;
    }
    planning_seconds_ += std::chrono::duration<double>(Clock::now() - started).count();
    execute(result->planned_trajectory);
  }

  RobotTrajectory cartesian(const Pose& pose, const RobotState* start = nullptr) {
    GetCartesianPath::Request request;
    request.header.frame_id = kWorld;
    request.group_name = kGroup;
    request.link_name = kHand;
    if (start) {
      request.start_state = *start;
    }
    request.start_state.is_diff = true;
    request.waypoints = {pose};
    request.max_step = .005;
    request.revolute_jump_threshold = .25;
    request.avoid_collisions = true;
    request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = .2;
    const auto answer = call_service<GetCartesianPath>("compute_cartesian_path", request);
    if (answer->error_code.val != MoveItErrorCodes::SUCCESS || answer->fraction < .99999) {
      std::ostringstream message;
      message << "Incomplete Cartesian path: " << std::fixed << std::setprecision(3) << answer->fraction;
      throw std::runtime_error(message.str());
    }
    return std::move(answer->solution);
  }

  void straight(const Pose& pose) {
    execute(cartesian(pose));
  }

  void execute(const RobotTrajectory& trajectory) {
    ExecuteTrajectory::Goal goal;
    goal.trajectory = trajectory;
    call_action<ExecuteTrajectory>("execute_trajectory", goal);
  }

  double grip(double width) {
    GripperCommand::Goal goal;
    goal.command.position = width / 2.;
    goal.command.max_effort = 2.5;
    auto left = rclcpp_action::create_client<GripperCommand>(this, "panda_hand_controller/gripper_cmd");
    auto right = rclcpp_action::create_client<GripperCommand>(this, "panda_right_finger_controller/gripper_cmd");
    if (!left->wait_for_action_server(10s) || !right->wait_for_action_server(10s)) {
      throw std::runtime_error("Finger controllers unavailable");
    }
    auto left_request = left->async_send_goal(goal);
    auto right_request = right->async_send_goal(goal);
    auto left_handle = wait(std::move(left_request));
    auto right_handle = wait(std::move(right_request));
    if (!left_handle || !right_handle) {
      throw std::runtime_error("Finger command rejected");
    }
    auto left_response = left->async_get_result(left_handle);
    auto right_response = right->async_get_result(right_handle);
    const auto left_result = wait(std::move(left_response));
    const auto right_result = wait(std::move(right_response));
    if (left_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        right_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      throw std::runtime_error("Finger motion failed");
    }
    return left_result.result->position + right_result.result->position;
  }

  gazebo_msgs::msg::EntityState observe(const std::string& name, const std::string& frame = kWorld) {
    GetEntityState::Request request;
    request.name = name;
    request.reference_frame = frame;
    const auto result = call_service<GetEntityState>("/gazebo/get_entity_state", request);
    if (!result->success) {
      throw std::runtime_error("Gazebo entity unavailable: " + name);
    }
    const auto& stamp = result->header.stamp;
    last_observation_time_ = stamp.sec + stamp.nanosec * 1e-9;
    return std::move(result->state);
  }

  void attach(bool attaching) {
    ApplyPlanningScene::Request request;
    request.scene.is_diff = true;
    request.scene.robot_state.is_diff = true;
    auto& attachment = request.scene.robot_state.attached_collision_objects.emplace_back();
    attachment.link_name = kHand;
    attachment.touch_links = {"panda_leftfinger", "panda_rightfinger"};
    attachment.object.id = kTarget;
    attachment.object.operation = attaching ? CollisionObject::ADD : CollisionObject::REMOVE;
    if (!call_service<ApplyPlanningScene>("apply_planning_scene", request)->success) {
      throw std::runtime_error("MoveIt rejected target attachment update");
    }
  }

  void log_contact_geometry(const char* phase) {
    for (const auto* name : {"panda::panda_leftfinger", "panda::panda_rightfinger", kTarget}) {
      const auto point = observe(name).pose.position;
      RCLCPP_INFO(get_logger(), "%s %s: (%.3f, %.3f, %.3f)", phase, name, point.x, point.y, point.z);
    }
  }

  static double joint_travel(const std::map<std::string, double>& current, const RobotState& start,
                             const RobotTrajectory& path) {
    double travel = 0.;
    const auto& joints = start.joint_state;
    const auto joint_count = std::min(joints.name.size(), joints.position.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
      if (is_arm_joint(joints.name[i])) {
        travel += std::abs(joints.position[i] - current.at(joints.name[i]));
      }
    }
    const auto& points = path.joint_trajectory.points;
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto& previous = points[i - 1].positions;
      const auto& next = points[i].positions;
      const auto joint_count = std::min(previous.size(), next.size());
      for (std::size_t j = 0; j < joint_count; ++j) {
        travel += std::abs(previous[j] - next[j]);
      }
    }
    return travel;
  }

  void select_approach(const Point& target, double box_yaw) {
    GetPlanningScene::Request query;
    query.components.components = 1023;  // All planning scene components in MoveIt Humble.
    const auto scene = call_service<GetPlanningScene>("get_planning_scene", query)->scene;
    const auto& initial = scene.robot_state;
    const auto& joints = initial.joint_state;
    std::map<std::string, double> current;
    const auto joint_count = std::min(joints.name.size(), joints.position.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
      current[joints.name[i]] = joints.position[i];
    }
    const auto elbow_joint = std::find(joints.name.begin(), joints.name.end(), "panda_joint3");
    const auto elbow_index = static_cast<std::size_t>(elbow_joint - joints.name.begin());
    if (elbow_index >= joint_count) {
      throw std::runtime_error("Planning scene is missing panda_joint3 position");
    }

    std::vector<Candidate> candidates;
    std::vector<double> seen_yaws;
    for (double yaw : {0., box_yaw, box_yaw - .22, box_yaw + .22}) {
      if (std::find(seen_yaws.begin(), seen_yaws.end(), yaw) != seen_yaws.end()) {
        continue;
      }
      seen_yaws.push_back(yaw);
      yaw_ = yaw;
      for (double elbow : {0., -.65, .65}) {
        auto seed = initial;
        seed.joint_state.position[elbow_index] = std::clamp(current.at("panda_joint3") + elbow, -2.8, 2.8);
        std::ostringstream description;
        description << std::fixed << std::setprecision(3) << "yaw=" << yaw
                    << std::setprecision(2) << " elbow_offset=" << elbow;
        const auto label = description.str();
        try {
          auto start = ik(hand_pose(target, kPregraspSetback), &seed);
          auto path = cartesian(hand_pose(target, kGraspSetback), &start);
          EvaluateCandidate::Request check;
          check.scene = scene;
          check.start = start;
          check.approach = path.joint_trajectory;
          if (strategy_ == "precheck") {
            const auto lift = lift_pose(target);
            check.loaded_waypoints = {lift, retreat_pose(lift)};
          }
          const auto result = call_service<EvaluateCandidate>("evaluate_candidate", check);
          if (!result->valid) {
            throw std::runtime_error(result->reason);
          }
          const double travel = joint_travel(current, start, path);
          candidates.push_back({result->clearance, travel, yaw, std::move(start), std::move(path), label});
        } catch (const std::runtime_error& error) {
          rejections_.push_back(label + ": " + error.what());
          RCLCPP_INFO(get_logger(), "Candidate rejected %s: %s", label.c_str(), error.what());
        }
      }
    }

    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
      return a.clearance == b.clearance ? a.travel < b.travel : a.clearance > b.clearance;
    });
    for (const auto& candidate : candidates) {
      yaw_ = candidate.yaw;
      try {
        move_joints(candidate.start.joint_state);
      } catch (const std::runtime_error& error) {
        rejections_.push_back(candidate.label + ": " + error.what());
        RCLCPP_INFO(get_logger(), "Candidate transfer rejected %s: %s", candidate.label.c_str(), error.what());
        continue;
      }
      execute(candidate.path);
      RCLCPP_INFO(get_logger(), "Selected %s clearance=%.4f travel=%.3f",
                  candidate.label.c_str(), candidate.clearance, candidate.travel);
      return;
    }
    throw std::runtime_error("All candidate approaches rejected");
  }

  void pick_place() {
    stage_ = "approach";
    const auto observed = observe(kTarget).pose;
    const auto& target = observed.position;
    ApplyPlanningScene::Request request;
    request.scene.is_diff = true;
    request.scene.robot_state.is_diff = true;
    auto& object = request.scene.world.collision_objects.emplace_back();
    object.id = kTarget;
    object.header.frame_id = kWorld;
    object.operation = CollisionObject::ADD;
    auto& box = object.primitives.emplace_back();
    box.type = SolidPrimitive::BOX;
    const auto& size = dimensions(kTarget);
    box.dimensions.assign(size.begin(), size.end());
    object.primitive_poses = {observed};
    if (!call_service<ApplyPlanningScene>("apply_planning_scene", request)->success) {
      throw std::runtime_error("Failed to refresh observed target");
    }
    grip(kOpenWidth);
    if (strategy_ == "fixed") {
      pregrasp(hand_pose(target, kPregraspSetback));
      straight(hand_pose(target, kGraspSetback));
    } else {
      const auto& q = observed.orientation;
      const auto yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z));
      select_approach(target, yaw);
    }
    RCLCPP_INFO(get_logger(), "Approach completed");
    log_contact_geometry("approach");

    stage_ = "grasp";
    grip(.060);
    grip(.050);
    const auto opening = grip(.030);
    if (opening < .033) {
      grip(kOpenWidth);
      straight(hand_pose(target, kPregraspSetback));
      throw MissedGrasp("Empty grasp: measured opening below 33 mm; reopened and retracted");
    }
    RCLCPP_INFO(get_logger(), "Jaw opening after closing: %.4f m", opening);
    log_contact_geometry("close");
    const auto before = observe(kTarget, "panda::panda_link7").pose.position;
    attach(true);
    const auto lift = lift_pose(target);
    straight(lift);
    const auto raised = observe(kTarget).pose.position;
    const auto after = observe(kTarget, "panda::panda_link7").pose.position;
    const auto slip = std::hypot(before.x - after.x, before.y - after.y, before.z - after.z);
    const auto raised_height = raised.z - target.z;
    if (std::abs(raised_height - kLiftHeight) > .008 || slip > .008) {
      grip(kOpenWidth);
      attach(false);
      straight(hand_pose(target, kPregraspSetback));
      std::ostringstream message;
      message << std::fixed << std::setprecision(4) << "Box moved " << raised_height
              << " m during 35 mm test lift, relative slip=" << slip << " m";
      throw MissedGrasp(message.str());
    }
    RCLCPP_INFO(get_logger(), "Verified physical lift: %.3f m", raised_height);
    grasped_ = true;

    stage_ = "withdraw";
    straight(retreat_pose(lift));
    stage_ = "transfer";
    const auto tray = observe("tray").pose.position;
    auto place = hand_pose(tray, kGraspSetback);
    place.position.z = tray.z + .06;
    pregrasp(place);
    log_contact_geometry("release");
    grip(kOpenWidth);
    attach(false);
    stage_ = "placement";
    verify_placement(tray);
    stage_ = "complete";
  }

  void verify_placement(const Point& tray) {
    const auto& box = dimensions(kTarget);
    const auto& tray_size = dimensions("tray");
    const auto deadline = Clock::now() + 4s;
    std::optional<double> stable_since;
    Point position;
    bool inside = false;
    bool still = false;
    while (Clock::now() < deadline) {
      const auto state = observe(kTarget);
      position = state.pose.position;
      const auto& q = state.pose.orientation;
      const double x = q.x, y = q.y, z = q.z, w = q.w;
      const double rotation[3][3] = {
          {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
          {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
          {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
      std::array<double, 3> extents{};
      for (std::size_t i = 0; i < extents.size(); ++i) {
        for (std::size_t j = 0; j < box.size(); ++j) {
          extents[i] += std::abs(rotation[i][j]) * box[j] / 2.;
        }
      }
      inside = std::abs(position.x - tray.x) + extents[0] <= tray_size[0] / 2. &&
               std::abs(position.y - tray.y) + extents[1] <= tray_size[1] / 2. &&
               std::abs(position.z - extents[2] - tray.z - tray_size[2] / 2.) <= .003;
      const auto& linear = state.twist.linear;
      const auto& angular = state.twist.angular;
      still = std::hypot(linear.x, linear.y, linear.z) < .005 &&
              std::hypot(angular.x, angular.y, angular.z) < .05;
      if (inside && still) {
        if (!stable_since) {
          stable_since = last_observation_time_;
        }
        if (last_observation_time_ - *stable_since >= .5) {
          RCLCPP_INFO(get_logger(), "Box stationary inside tray for 0.5 simulation seconds");
          return;
        }
      } else {
        stable_since.reset();
      }
      std::this_thread::sleep_for(50ms);
    }

    std::ostringstream message;
    message << "Tray check failed: position=(" << position.x << ", " << position.y << ", " << position.z
            << "), inside=" << std::boolalpha << inside << ", still=" << still
            << ", stamp=" << last_observation_time_ << ", stable_since=";
    if (stable_since) {
      message << *stable_since;
    } else {
      message << "None";
    }
    throw std::runtime_error(message.str());
  }

  std::string strategy_;
  std::string stage_ = "initialization";
  std::vector<std::string> rejections_;
  double yaw_ = 0.;
  double planning_seconds_ = 0.;
  double last_observation_time_ = 0.;
  bool grasped_ = false;
  int attempts_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<FixedGrasp>();
    std::optional<std::string> error;
    try {
      node->run();
    } catch (const std::exception& exception) {
      error = exception.what();
      RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
      status = 1;
    }
    std::cout << "TRIAL_RESULT " << node->result(error).dump() << std::endl;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(rclcpp::get_logger("fixed_grasp"), "%s", exception.what());
    status = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return status;
}
