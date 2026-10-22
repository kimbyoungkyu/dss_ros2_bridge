# dss_nav2.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.expanduser("~/.dss/navigation/current")

    # apt로 설치한 Humble Nav2 사용
    bringup_dir = "/opt/ros/humble/share/nav2_bringup"

    # my_nav2 패키지에 복사한 Nav2 설정
    package_dir = get_package_share_directory("my_nav2")
    params_file = os.path.join(package_dir, "config", "nav2_params.yaml")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": os.path.join(config_dir, "final.yaml"),
                "params_file": params_file,
                "use_sim_time": "True",
                "autostart": "True",
                "slam": "False",
                "use_composition": "False",
            }.items(),
        ),
    ])