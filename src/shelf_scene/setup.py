from setuptools import setup

setup(
    name="shelf_scene", version="0.1.0", packages=["shelf_scene"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/shelf_scene"]),
        ("share/shelf_scene", ["package.xml"]),
        ("share/shelf_scene/launch", ["launch/baseline.launch.py"]),
    ],
    entry_points={"console_scripts": [
        "publish_scene = shelf_scene.publisher:main",
        "gazebo_world = shelf_scene.world:main",
        "fixed_grasp = shelf_scene.fixed_grasp:main",
    ]},
)
