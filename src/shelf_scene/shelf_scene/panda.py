"""Build the Gazebo robot description using installed Panda resources."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


def share(name):
    return Path(get_package_share_directory(name))


def robot_description(controller_file):
    description = share("moveit_resources_panda_description")
    config = share("moveit_resources_panda_moveit_config") / "config"
    franka = share("franka_description")
    robot = ET.parse(description / "urdf/panda.urdf").getroot()
    inertials = {}
    for directory in ("robots/fer", "end_effectors/franka_hand"):
        inertials.update(yaml.safe_load((franka / directory / "inertials.yaml").read_text()))
    for link in robot.findall("link"):
        name = link.get("name").removeprefix("panda_")
        if name == "link8":  # Fixed flange frame
            continue
        data = inertials[name]
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", data["origin"])
        ET.SubElement(inertial, "mass", value=str(data["mass"]))
        ET.SubElement(inertial, "inertia", {"i"+k: str(v) for k, v in data["inertia"].items()})
    # Plan and simulate against flat finger pads for stable ODE box contacts.
    for finger, sign in (("panda_leftfinger", 1), ("panda_rightfinger", -1)):
        link = robot.find(f"link[@name='{finger}']")
        link.remove(link.find("collision"))
        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin", xyz=f"0 {sign * .008} .029")
        ET.SubElement(ET.SubElement(collision, "geometry"), "box", size=".020 .016 .048")
        surface = ET.SubElement(robot, "gazebo", reference=finger)
        for tag, value in (("mu1", "1.0"), ("mu2", "1.0"), ("minDepth", "0.0001")):
            ET.SubElement(surface, tag).text = value
    ET.SubElement(ET.SubElement(robot, "gazebo"), "plugin",
                  name="shelf_grasp", filename="libshelf_grasp.so")
    ET.SubElement(robot, "link", name="world")
    mount = ET.SubElement(robot, "joint", name="mount", type="fixed")
    ET.SubElement(mount, "parent", link="world")
    ET.SubElement(mount, "child", link="panda_link0")
    control = ET.SubElement(robot, "ros2_control", name="Panda", type="system")
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "gazebo_ros2_control/GazeboSystem"
    positions = yaml.safe_load((config / "initial_positions.yaml").read_text())["initial_positions"]
    positions.update(panda_finger_joint1=0.04, panda_finger_joint2=0.04)
    for name, position in positions.items():
        joint = ET.SubElement(control, "joint", name=name)
        ET.SubElement(joint, "command_interface", name="effort" if "finger" in name else "position")
        state = ET.SubElement(joint, "state_interface", name="position")
        ET.SubElement(state, "param", name="initial_value").text = str(position)
        ET.SubElement(joint, "state_interface", name="velocity")
    controllers = yaml.safe_load((config / "ros2_controllers.yaml").read_text())
    controllers["controller_manager"]["ros__parameters"]["update_rate"] = 500
    hand = "panda_hand_controller"
    controllers["controller_manager"]["ros__parameters"][hand]["type"] = (
        "effort_controllers/GripperActionController")
    controllers[hand]["ros__parameters"].update(
        goal_tolerance=.001, allow_stalling=True, stall_timeout=0.4, stall_velocity_threshold=0.15,
        gains={"panda_finger_joint1": {"p": 450.0, "i": 0.0, "d": 2.0}})
    other = "panda_right_finger_controller"
    controllers["controller_manager"]["ros__parameters"][other] = dict(
        controllers["controller_manager"]["ros__parameters"][hand])
    right = dict(controllers[hand]["ros__parameters"])
    right.update(joint="panda_finger_joint2",
                 gains={"panda_finger_joint2": dict(right["gains"]["panda_finger_joint1"])})
    controllers[other] = {"ros__parameters": right}
    controller_file.write_text(yaml.safe_dump(controllers))
    plugin = ET.SubElement(ET.SubElement(robot, "gazebo"), "plugin",
                           name="gazebo_ros2_control", filename="libgazebo_ros2_control.so")
    ET.SubElement(plugin, "parameters").text = str(controller_file)
    return ET.tostring(robot, encoding="unicode")
