from setuptools import setup

setup(
    name="hsl_planning",
    version="0.1.0",
    packages=["hsl_planning"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hsl_planning"]),
        ("share/hsl_planning", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Just Robotics",
    maintainer_email="dev@just-robotics.ru",
    description="3D obstacle projection and role-aware path planning",
    license="Apache-2.0",
    entry_points={"console_scripts": ["trajectory_planner = hsl_planning.node:main"]},
)
