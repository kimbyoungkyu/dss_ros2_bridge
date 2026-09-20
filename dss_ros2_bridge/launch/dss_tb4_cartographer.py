# dss_cartographer.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = "/home/dss/.dss/navigation/current"

    return LaunchDescription([
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", "dss_tb4_2d.lua",
            ],
            parameters=[{
                "use_sim_time": True,
            }],
        ),

        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            arguments=[
                "-resolution", "0.05",
                "-publish_period_sec", "1.0",
            ],
            parameters=[{
                "use_sim_time": True,
            }],
        ),
    ])