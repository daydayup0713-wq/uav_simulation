from setuptools import setup, find_packages

setup(name='uav_lab_localization', version='0.4.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_lab_localization']),
                  ('share/uav_lab_localization', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='UAV Lab',
      maintainer_email='maintainers@example.invalid', description='CPU LIO and SLAM laboratory',
      license='Apache-2.0', entry_points={'console_scripts': [
          'slamctl = uav_lab_localization.cli:main',
          'localization = uav_lab_localization.localization_node:main']})
