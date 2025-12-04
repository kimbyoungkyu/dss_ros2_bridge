from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera
        Node(
            package='dss_ros2_bridge',
            executable='DSSToROSImageNode',
            name='Image',
            output='screen'
        ),

        # IMU
        Node(
            package='dss_ros2_bridge',
            executable='DSSToROSIMUNode',
            name='IMU',
            output='screen'
        ),

        # LiDAR
        Node(
            package='dss_ros2_bridge',
            executable='DSSToROSPointCloudNode',
            name='PointCloud',
            output='screen'
        ),
        
        # GPS
        #Node(
         #   package='dss_ros2_bridge',
          #  executable='DSSToGPSNode',
           # name='GPS',
            #output='screen'
        #),
        
        # DSSDemo
        #Node(
         #   package='dss_ros2_bridge',
          #  executable='DSSDemoNode',
           # name='Demo',
            #output='screen'
        #),
        
    ])
