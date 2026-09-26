from setuptools import setup

setup(
    name="hsl_decision",
    version="0.1.0",
    packages=["hsl_decision"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hsl_decision"]),
        ("share/hsl_decision", ["package.xml"]),
        ("share/hsl_decision/config", ["config/decision.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Just Robotics",
    maintainer_email="dev@just-robotics.ru",
    description="Role-aware behavior selection for HSL",
    license="Apache-2.0",
    entry_points={"console_scripts": ["decision_manager = hsl_decision.node:main"]},
)
