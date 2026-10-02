from setuptools import setup

setup(name='hsl_perception', version='0.2.0', packages=['hsl_perception'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/hsl_perception']),
                  ('share/hsl_perception', ['package.xml', 'SOURCE.md'])],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      maintainer='Just Robotics', maintainer_email='dev@just-robotics.ru',
      description='Shape-based LiDAR opponent detection and Kalman tracking',
      license='Apache-2.0',
      entry_points={'console_scripts': ['opponent_detector = hsl_perception.node:main']})
