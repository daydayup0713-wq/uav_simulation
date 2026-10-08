from setuptools import setup, find_packages

setup(name='uav_lab_experiments', version='0.4.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_lab_experiments']),
                  ('share/uav_lab_experiments', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='UAV Lab',
      maintainer_email='maintainers@example.invalid', description='Reproducible multi-backend experiments',
      license='Apache-2.0', entry_points={'console_scripts': ['experiments = uav_lab_experiments.cli:main',
                      'timed_sensors = uav_lab_experiments.timed_sensor_node:main',
                      'observatory = uav_lab_experiments.observation_node:main']})
