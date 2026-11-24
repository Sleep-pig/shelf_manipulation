#include <chrono>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "shelf_scene/layout.hpp"

namespace
{
void apply_scene(
  const rclcpp::Node::SharedPtr & node, const moveit_msgs::msg::PlanningScene & scene)
{
  using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;
  auto client = node->create_client<ApplyPlanningScene>("apply_planning_scene");
  while (rclcpp::ok() && !client->wait_for_service(std::chrono::seconds(1))) {
  }
  if (!rclcpp::ok()) {
    return;
  }

  auto request = std::make_shared<ApplyPlanningScene::Request>();
  request->scene = scene;
  auto response = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, response) != rclcpp::FutureReturnCode::SUCCESS) {
    return;
  }
  if (!response.get()->success) {
    throw std::runtime_error("MoveIt rejected the shelf scene");
  }
  RCLCPP_INFO(node->get_logger(), "Shelf geometry applied to MoveIt");
}

void publish_scene(const rclcpp::Node::SharedPtr & node)
{
  const auto seed = node->declare_parameter<std::int64_t>("seed", 0);
  const double width = node->declare_parameter("width", .36);
  const double depth = node->declare_parameter("depth", .32);
  const bool apply = node->declare_parameter("apply_to_move_group", false);

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  visualization_msgs::msg::MarkerArray markers;

  for (const auto & box : shelf_scene::layout(seed, width, depth)) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = box.center[0];
    pose.position.y = box.center[1];
    pose.position.z = box.center[2];
    pose.orientation.z = std::sin(box.yaw / 2);
    pose.orientation.w = std::cos(box.yaw / 2);

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.assign(box.size.begin(), box.size.end());

    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = box.name;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    scene.world.collision_objects.push_back(object);

    visualization_msgs::msg::Marker marker;
    marker.header = object.header;
    marker.ns = "shelf";
    marker.id = static_cast<std::int32_t>(markers.markers.size());
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = box.size[0];
    marker.scale.y = box.size[1];
    marker.scale.z = box.size[2];
    const bool target = box.name == "target";
    marker.color.r = target ? 1.f : .35f;
    marker.color.g = .55f;
    marker.color.b = target ? .12f : .70f;
    marker.color.a = target ? 1.f : .65f;
    markers.markers.push_back(marker);
  }

  const auto qos = rclcpp::QoS(1).transient_local();
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", qos);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("shelf/markers", qos);
  scene_pub->publish(scene);
  marker_pub->publish(markers);
  RCLCPP_INFO(node->get_logger(), "Published %zu boxes, seed=%" PRId64, markers.markers.size(), seed);

  if (apply) {
    apply_scene(node, scene);
  }
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    publish_scene(std::make_shared<rclcpp::Node>("shelf_scene"));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("shelf_scene"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
