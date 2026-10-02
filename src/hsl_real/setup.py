from setuptools import setup
from glob import glob
setup(name='hsl_real', version='0.1.0', packages=['hsl_real'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/hsl_real']),
                  ('share/hsl_real', ['package.xml']),
                  ('share/hsl_real/launch', glob('launch/*.launch.py')),
                  ('share/hsl_real/config', glob('config/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Just Robotics', maintainer_email='dev@just-robotics.ru',
      description='Real robot integration', license='Apache-2.0',
      entry_points={'console_scripts': ['real_observations=hsl_real.observations:main',
                                       'real_lidar_filter=hsl_real.lidar_filter:main',
                                       'real_match=hsl_real.match:main',
                                       'localization_monitor=hsl_real.localization:main',
                                       'real_manual_gate=hsl_real.manual:main']})
