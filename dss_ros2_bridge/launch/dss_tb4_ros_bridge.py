import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common_params = {
        "use_sim_time": True,
    }

    nav_control_params = {
        "storage_directory": os.path.expanduser("~/.dss/navigation"),
    }

    return LaunchDescription([
        # DSS 센서 → ROS2 및 /cmd_vel → DSS
        Node(
            package="dss_ros2_bridge",
            executable="DSS_TB4_SimToROSBridgeNode",
            name="dss_tb4_ros_bridge",
            output="screen",
            parameters=[common_params],
        ),

        # dss.nav.control 요청 처리 및 설정·지도 파일 저장
        Node(
            package="dss_ros2_bridge",
            executable="DSS_TB4_ROSNavControlNode",
            name="TB4ROSNavControl",
            output="screen",
            parameters=[
                common_params,
                nav_control_params,
            ],
        ),
    ])