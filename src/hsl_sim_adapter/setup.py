from setuptools import setup

setup(
    name="hsl_sim_adapter", version="0.1.0", packages=["hsl_sim_adapter"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hsl_sim_adapter"]),
        ("share/hsl_sim_adapter", ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Just Robotics", maintainer_email="dev@just-robotics.ru",
    description="Gazebo-only observation and scenario adapters",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "sim_observations = hsl_sim_adapter.observations:main",
        "scripted_opponent = hsl_sim_adapter.scripted_opponent:main",
        "match_metrics = hsl_sim_adapter.match_metrics:main",
    ]},
)
