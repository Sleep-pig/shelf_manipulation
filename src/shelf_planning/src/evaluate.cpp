#include <cmath>
#include <limits>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <shelf_planning/srv/evaluate_candidate.hpp>

using Service = shelf_planning::srv::EvaluateCandidate;

void evaluate(const moveit::core::RobotModelPtr& model, const Service::Request& req,
              Service::Response& res) {
  planning_scene::PlanningScene scene(model);
  if (!scene.setPlanningSceneMsg(req.scene) || req.approach.points.empty()) {
    res.reason = "missing scene or approach";
    return;
  }
  auto& state = scene.getCurrentStateNonConst();
  moveit::core::robotStateMsgToRobotState(req.start, state, false);
  auto group = model->getJointModelGroup("panda_arm");
  const auto& names = req.approach.joint_names;
  if (names != group->getVariableNames()) {
    res.reason = "unexpected arm joints";
    return;
  }
  res.clearance = std::numeric_limits<double>::max();
  for (const auto& point : req.approach.points) {
    if (point.positions.size() != names.size()) {
      res.reason = "incomplete trajectory point";
      return;
    }
    state.setVariablePositions(names, point.positions);
    state.update();
    if (!state.satisfiesBounds(group) || scene.isStateColliding(state)) {
      res.reason = "approach collision or joint limit";
      return;
    }
    res.clearance = std::min(res.clearance, scene.distanceToCollision(state));
  }
  if (!req.loaded_waypoints.empty()) {
    moveit_msgs::msg::AttachedCollisionObject object;
    object.link_name = "panda_hand";
    object.object.id = "target";
    object.touch_links = {"panda_leftfinger", "panda_rightfinger"};
    if (!scene.processAttachedCollisionObjectMsg(object)) {
      res.reason = "unable to attach target in scene copy";
      return;
    }
    for (const auto& end : req.loaded_waypoints) {
      auto origin = state.getGlobalLinkTransform("panda_hand").translation().eval();
      Eigen::Vector3d dest(end.position.x, end.position.y, end.position.z);
      int steps = std::max(1, static_cast<int>(std::ceil((dest-origin).norm() / .005)));
      for (int i = 1; i <= steps; ++i) {
        auto pose = end;
        Eigen::Vector3d xyz = origin + (dest-origin) * (double(i)/steps);
        pose.position.x = xyz.x(); pose.position.y = xyz.y(); pose.position.z = xyz.z();
        auto previous = state;
        if (!state.setFromIK(group, pose, "panda_hand", .06)) {
          res.reason = "loaded retreat IK";
          return;
        }
        state.update();
        if (!state.satisfiesBounds(group) || state.distance(previous, group) > .65 ||
            scene.isStateColliding(state)) {
          res.reason = "loaded retreat collision, discontinuity or joint limit";
          return;
        }
      }
    }
  }
  res.valid = true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("shelf_candidate_evaluator",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_model_loader::RobotModelLoader loader(node);
  auto model = loader.getModel();
  if (!model || !model->getJointModelGroup("panda_arm")) return 1;
  auto service = node->create_service<Service>("evaluate_candidate",
      [model](Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
        evaluate(model, *req, *res);
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
