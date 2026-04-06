import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 및 Xacro 설정 
    pkg_path = get_package_share_directory('hyper_pupper_complete')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mini-pupper.urdf.xacro')
    
    # Xacro를 XML 텍스트로 변환 
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. 노드 정의
    # Robot State Publisher: URDF 정보를 TF로 발행 [cite: 134, 136]
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'publish_frequency': 30.0 # 기존 30Hz 설정 반영 
        }]
    )

    # Joint State Publisher GUI: 슬라이더로 관절 조작
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz2: 설정 파일 로드 [cite: 135]
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'added_sensors.rviz')]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz
    ])