from setuptools import setup, find_packages
from glob import glob
setup(name='uav_lab_bridge', version="0.2.0", packages=find_packages(), data_files=[("share/ament_index/resource_index/packages", ["resource/uav_lab_bridge"]), ("share/uav_lab_bridge", ["package.xml"]), ("share/uav_lab_bridge/launch", glob("launch/*.launch.py"))], install_requires=["setuptools"], zip_safe=True, maintainer="UAV Lab", maintainer_email="maintainers@example.invalid", description="PX4 simulation lab", license="Apache-2.0", entry_points={"console_scripts": ['bridge = uav_lab_bridge.node:main']})
