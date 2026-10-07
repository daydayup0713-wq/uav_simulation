from setuptools import setup,find_packages

setup(name='uav_lab_navigation',version='0.4.0',packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages',['resource/uav_lab_navigation']),('share/uav_lab_navigation',['package.xml'])],
    install_requires=['setuptools'],zip_safe=True,maintainer='UAV Lab',maintainer_email='maintainers@example.invalid',
    description='Three-dimensional observed-space navigation',license='Apache-2.0',
    entry_points={'console_scripts':['navigation = uav_lab_navigation.node:main','navctl = uav_lab_navigation.cli:main']})
