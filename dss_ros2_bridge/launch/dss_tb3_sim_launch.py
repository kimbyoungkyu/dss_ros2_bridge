import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    common_params = {"use_sim_time": True}

    nav_control_params = {
        "storage_directory": os.path.expanduser("~/.dss/navigation"),
    }

    param_bridge_params = {
        "target_node": "/dss_bridge",
        "nats_url": "nats://127.0.0.1:4222",
        "service_timeout_ms": 3000,
    }

    start_dss = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="dss_ros2_bridge",
                executable="DSS_TB3_SimToROSBridgeNode",
                name="dss_bridge",
                output="screen",
                parameters=[common_params],
            ),
            Node(
                package="dss_ros2_bridge",
                executable="DSS_TB3_ROSNavControlNode",
                name="dss_tb3_ros_nav_control",
                output="screen",
                parameters=[common_params, nav_control_params],
            ),
            Node(
                package="dss_ros2_bridge",
                executable="dss_param_nats_bridge_node",
                name="dss_param_nats_bridge",
                output="screen",
                parameters=[param_bridge_params],
            ),
        ],
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=["dss-tb3b", "start"],
            name="dss_tb3_suite",
            output="screen",
        ),
        start_dss,
    ])