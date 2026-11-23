from pathlib import Path
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from shelf_scene.panda import robot_description, share
from shelf_scene.world import world


def planning_nodes(description):
    config = share("moveit_resources_panda_moveit_config") / "config"
    semantic = ET.parse(config / "panda.srdf").getroot()
    # The simulated URDF already contains the fixed world mount.
    semantic.remove(semantic.find("virtual_joint"))

    def settings(filename):
        return yaml.safe_load((config / filename).read_text())

    ompl = settings("ompl_planning.yaml")
    ompl["panda_arm"]["longest_valid_segment_fraction"] = 0.001

    parameters = {
                    "use_sim_time": True,
                    "robot_description": description,
                    "robot_description_semantic": ET.tostring(semantic, encoding="unicode"),
                    "robot_description_kinematics": settings("kinematics.yaml"),
                    "robot_description_planning": settings("joint_limits.yaml"),
                    "planning_pipelines": ["ompl"],
                    "default_planning_pipeline": "ompl",
                    "ompl": ompl,
                }
    return [Node(package="moveit_ros_move_group", executable="move_group",
                 parameters=[parameters, settings("moveit_controllers.yaml")]),
            Node(package="shelf_planning", executable="evaluate", parameters=[parameters])]



def start(context):
    seed = int(LaunchConfiguration("seed").perform(context))
    width = float(LaunchConfiguration("width").perform(context))
    depth = float(LaunchConfiguration("depth").perform(context))
    directory = tempfile.TemporaryDirectory(prefix="shelf-world-")
    path = os.path.join(directory.name, "scene.world")
    ET.ElementTree(world(seed, width, depth)).write(path)
    # Retain the temporary world for the lifetime of the launch.
    description = robot_description(Path(directory.name) / "controllers.yaml")
    gazebo = ExecuteProcess(cmd=["gzserver", "-s", "libgazebo_ros_init.so",
                                 "-s", "libgazebo_ros_factory.so", path],
                            additional_env={"GAZEBO_MODEL_PATH": str(share("moveit_resources_panda_description").parent)
                                            + os.pathsep + os.environ.get("GAZEBO_MODEL_PATH", "")})
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: directory.cleanup())])),
        gazebo,
        *planning_nodes(description),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}]),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-topic", "robot_description", "-entity", "panda"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "panda_arm_controller", "panda_hand_controller", "panda_right_finger_controller",
                        "--controller-manager-timeout", "60"]),
        Node(package="shelf_scene", executable="publish_scene", parameters=[{"seed": seed, "width": width, "depth": depth, "apply_to_move_group": True}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("seed", default_value="7"),
                              DeclareLaunchArgument("width", default_value="0.36"),
                              DeclareLaunchArgument("depth", default_value="0.32"),
                              OpaqueFunction(function=start)])
