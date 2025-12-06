from setuptools import find_packages
from setuptools import setup

setup(
    name='dss_ros2_bridge',
    version='1.0.0',
    packages=find_packages(
        include=('dss_ros2_bridge', 'dss_ros2_bridge.*')),
)
