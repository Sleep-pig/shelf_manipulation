import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray


from .layout import layout


def messages(seed, width=.36, depth=.32):
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    markers = MarkerArray()
    for index, (name, size, center, yaw) in enumerate(layout(seed, width, depth)):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.z = math.sin(yaw / 2)
        pose.orientation.w = math.cos(yaw / 2)
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
        obj = CollisionObject(id=name, primitives=[box], primitive_poses=[pose])
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD
        scene.world.collision_objects.append(obj)
        marker = Marker(header=obj.header, ns="shelf", id=index,
                        type=Marker.CUBE, pose=pose)
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b = (
            (1., .55, .12) if name == "target" else (.35, .55, .70))
        marker.color.a = 1. if name == "target" else .65
        markers.markers.append(marker)
    return scene, markers


def main():
    rclpy.init()
    node = rclpy.create_node("shelf_scene")
    try:
        seed = node.declare_parameter("seed", 0).value
        width = node.declare_parameter("width", .36).value
        depth = node.declare_parameter("depth", .32).value
        scene, markers = messages(seed, width, depth)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scene_pub = node.create_publisher(PlanningScene, "planning_scene", qos)
        marker_pub = node.create_publisher(MarkerArray, "shelf/markers", qos)
        scene_pub.publish(scene)
        marker_pub.publish(markers)
        node.get_logger().info(f"Published {len(markers.markers)} boxes, seed={seed}")
        if node.declare_parameter("apply_to_move_group", False).value:
            client = node.create_client(ApplyPlanningScene, "apply_planning_scene")
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                pass
            if not rclpy.ok():
                return
            request = client.call_async(ApplyPlanningScene.Request(scene=scene))
            rclpy.spin_until_future_complete(node, request)
            if not request.result().success:
                raise RuntimeError("MoveIt rejected the shelf scene")
            node.get_logger().info("Shelf geometry applied to MoveIt")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
