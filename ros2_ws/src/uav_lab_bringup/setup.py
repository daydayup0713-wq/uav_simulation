from setuptools import setup, find_packages
from glob import glob
setup(name='uav_lab_bringup', version="0.1.0", packages=find_packages(), data_files=[("share/ament_index/resource_index/packages", ["resource/uav_lab_bringup"]), ("share/uav_lab_bringup", ["package.xml"]), ("share/uav_lab_bringup/launch", glob("launch/*.launch.py"))], install_requires=["setuptools"], zip_safe=True, maintainer="UAV Lab", maintainer_email="maintainers@example.invalid", description="PX4 simulation lab", license="Apache-2.0", entry_points={"console_scripts": []})
