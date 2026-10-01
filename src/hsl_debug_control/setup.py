from glob import glob
from setuptools import setup

setup(
    name="hsl_debug_control", version="0.1.0", packages=["hsl_debug_control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hsl_debug_control"]),
        ("share/hsl_debug_control", ["package.xml"]),
        ("share/hsl_debug_control/launch", glob("launch/*.launch.py")),
        ("share/hsl_debug_control/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Just Robotics", maintainer_email="dev@just-robotics.ru",
    description="Safety gate and RViz for native MPPI",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "motion_gate = hsl_debug_control.motion_gate:main",
    ]},
)
