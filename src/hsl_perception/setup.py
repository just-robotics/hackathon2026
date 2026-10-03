from setuptools import setup

setup(name='hsl_perception', version='0.2.0', packages=['hsl_perception'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/hsl_perception']),
                  ('share/hsl_perception', ['package.xml', 'SOURCE.md'])],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      maintainer='Just Robotics', maintainer_email='dev@just-robotics.ru',
      description='Shared LiDAR decoding, static-map masking and obstacle clustering',
      license='Apache-2.0',
      entry_points={})
