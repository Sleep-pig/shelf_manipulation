import math
import random

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray


def layout(seed):
    rng = random.Random(seed)
    return [
        ("floor", (.32, .40, .02), (.53, 0., .20), 0.),
        ("ceiling", (.32, .40, .02), (.53, 0., .48), 0.),
        ("left", (.32, .02, .26), (.53, .19, .34), 0.),
        ("right", (.32, .02, .26), (.53, -.19, .34), 0.),
        ("back", (.02, .36, .26), (.68, 0., .34), 0.),
        ("target", (.07, .04, .06),
         (rng.uniform(.44, .59), rng.uniform(-.07, .03), .24),
         rng.uniform(-.6, .6)),
        ("obstacle", (.08, .05, .12), (.49, .13, .27), 0.),
        ("tray", (.20, .18, .02), (.32, -.34, .15), 0.),
    ]


def messages(seed):
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    markers = MarkerArray()
    for index, (name, size, center, yaw) in enumerate(layout(seed)):
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
        scene, markers = messages(seed)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scene_pub = node.create_publisher(PlanningScene, "planning_scene", qos)
        marker_pub = node.create_publisher(MarkerArray, "shelf/markers", qos)
        scene_pub.publish(scene)
        marker_pub.publish(markers)
        node.get_logger().info(f"Published {len(markers.markers)} boxes, seed={seed}")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
