import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    common_params = {
        "use_sim_time": True,
    }

    nav_control_params = {
        "storage_directory": os.path.expanduser("~/.dss/navigation"),
    }

    param_bridge_params = {
        "target_node": "/dss_bridge",
        "nats_url": "nats://127.0.0.1:4222",
        "service_timeout_ms": 3000,
    }

    return LaunchDescription([
        # DSS 센서 → ROS 2
        # ROS 2 /cmd_vel → DSS
        #
        # 파라미터 관리 대상 노드이므로 이름을 /dss_bridge로 지정
        Node(
            package="dss_ros2_bridge",
            executable="DSS_TB4_SimToROSBridgeNode",
            name="dss_bridge",
            output="screen",
            parameters=[
                common_params,
            ],
        ),

        # dss.nav.control 요청 처리
        # Cartographer 실행 및 설정·지도 파일 관리
        Node(
            package="dss_ros2_bridge",
            executable="DSS_TB4_ROSNavControlNode",
            name="dss_tb4_ros_nav_control",
            output="screen",
            parameters=[
                common_params,
                nav_control_params,
            ],
        ),

        # NATS JSON ↔ ROS 2 동적 파라미터 브리지
        Node(
            package="dss_ros2_bridge",
            executable="dss_param_nats_bridge_node",
            name="dss_param_nats_bridge",
            output="screen",
            parameters=[
                param_bridge_params,
            ],
        ),
        
        ExecuteProcess(
            cmd=["nats-server"],
            name="nats-server",
            output="screen",
        ),        
        
        
        ExecuteProcess(
            cmd=["dss-tb4-control-panel"],
            name="dss_tb4_control_panel",
            output="screen",
        ),

        ExecuteProcess(
            cmd=["dss-tb4-sim-academy"],
            name="dss_tb4_sim_academy",
            output="screen",
        ),        
        
        
        
        
    ])