from setuptools import setup
from glob import glob

setup(
    name="hsl_sim_adapter", version="0.1.0", packages=["hsl_sim_adapter"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hsl_sim_adapter"]),
        ("share/hsl_sim_adapter", ["package.xml"]),
        ("share/hsl_sim_adapter/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Just Robotics", maintainer_email="dev@just-robotics.ru",
    description="Gazebo-only observation and scenario adapters",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "sim_observations = hsl_sim_adapter.observations:main",
        "match_metrics = hsl_sim_adapter.match_metrics:main",
        "duel_referee = hsl_sim_adapter.duel_referee:main",
    ]},
)
