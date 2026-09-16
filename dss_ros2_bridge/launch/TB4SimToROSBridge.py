from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    #  모든 노드가 사용할 공통 파라미터
    common_params = {
        "use_sim_time": True,   #sim time 사용
    }

    return LaunchDescription([
        
        # Clock (필요하면 활성화)
        Node(
            package='dss_ros2_bridge',
            executable='DSS_TB4_SimToROSBridgeNode',
            name='TB4SimToROSBridge',
            output='screen',
            parameters=[common_params],
        ),
        

        # DSS Demo
        # Node(
        #     package='dss_ros2_bridge',
        #     executable='DSSDemoNode',
        #     name='Demo',
        #     output='screen',
        #     parameters=[common_params],
        # ),
    ])
