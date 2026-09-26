from setuptools import setup

setup(
    name="hsl_debug_control", version="0.1.0", packages=["hsl_debug_control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hsl_debug_control"]),
        ("share/hsl_debug_control", ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Just Robotics", maintainer_email="dev@just-robotics.ru",
    description="Replaceable safety-gated path follower for Gazebo tests",
    license="Apache-2.0",
    entry_points={"console_scripts": ["debug_follower = hsl_debug_control.node:main"]},
)
