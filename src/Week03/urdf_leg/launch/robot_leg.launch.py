import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    urdf_leg_share_dir = get_package_share_directory('urdf_leg')
    urdf_file_path = os.path.join(urdf_leg_share_dir, 'urdf', 'robot_leg.urdf')
    rviz_config_path = os.path.join(urdf_leg_share_dir, 'rviz', 'default.rviz')

    # Read the URDF file into a string
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Robot State Publisher: Broadcasts the TF tree and robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 2. Joint State Publisher GUI: Opens the window with sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # 3. RViz2: Opens the visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path] # "-d" 옵션으로 설정 파일 경로를 전달!
        )
    ])