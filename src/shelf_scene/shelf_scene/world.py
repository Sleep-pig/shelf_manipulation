"""Generate a Gazebo Classic world from the planning-scene geometry."""
import argparse
import xml.etree.ElementTree as ET

from .layout import layout


def world(seed, width=.36, depth=.32):
    sdf = ET.Element("sdf", version="1.6")
    scene = ET.SubElement(sdf, "world", name="shelf")
    state = ET.SubElement(scene, "plugin", name="gazebo_ros_state", filename="libgazebo_ros_state.so")
    ET.SubElement(ET.SubElement(state, "ros"), "namespace").text = "/gazebo"
    for name, size, center, yaw in layout(seed, width, depth):
        model = ET.SubElement(scene, "model", name=name)
        dynamic = name == "target"
        ET.SubElement(model, "static").text = "false" if dynamic else "true"
        ET.SubElement(model, "pose").text = " ".join(map(str, (*center, 0, 0, yaw)))
        link = ET.SubElement(model, "link", name="body")
        for kind in ("collision", "visual"):
            shape = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(shape, "geometry"), "box")
            ET.SubElement(box, "size").text = " ".join(map(str, size))
        if dynamic:
            # Damp ODE contact correction at rest on the shelf and tray.
            contact = ET.SubElement(ET.SubElement(link.find("collision"), "surface"), "contact")
            ode = ET.SubElement(contact, "ode")
            ET.SubElement(ode, "max_vel").text = "0.01"
            ET.SubElement(ode, "min_depth").text = "0.0001"
            ET.SubElement(ode, "kp").text = "100000"
            ET.SubElement(ode, "kd").text = "10"
            mass = 0.1
            x, y, z = size
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass").text = str(mass)
            inertia = ET.SubElement(inertial, "inertia")
            for axis, a, b in (("ixx", y, z), ("iyy", x, z), ("izz", x, y)):
                ET.SubElement(inertia, axis).text = str(mass * (a*a + b*b) / 12)
    ET.indent(sdf)
    return sdf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--width", type=float, default=.36)
    parser.add_argument("--depth", type=float, default=.32)
    args = parser.parse_args()
    print(ET.tostring(world(args.seed, args.width, args.depth), encoding="unicode"))
