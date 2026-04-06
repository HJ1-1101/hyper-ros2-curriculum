import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, AppendEnvironmentVariable,
    RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('hyper_pupper_complete')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mini-pupper.urdf.xacro')
    # Harmonic용 패키지 이름은 ros_gz_sim입니다.
    gz_sim_path = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(get_package_share_directory('hyper_pupper_complete'), 'worlds', 'study.sdf')

    # [핵심] Gazebo에게 메쉬 경로를 알려주는 환경 변수 설정
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..')
    )

    # 1. Xacro 파싱
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Gazebo 실행 설정 (Harmonic용)
    # '-r'은 실행 즉시 시뮬레이션을 시작(run)하라는 뜻이며, 'empty.sdf'는 빈 월드를 의미합니다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 3. 로봇 소환 노드 (Harmonic 전용 'create' 노드)
    # -string 옵션을 써서 파싱된 xacro(xml) 내용을 직접 전달합니다.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hyper_pupper_complete_robot',
            '-string', robot_description_raw, # -topic 대신 -string이 더 확실합니다.
            '-x', '0', '-y', '0', '-z', '0.2'  # 바닥에 끼지 않게 살짝 띄워서 소환
        ],
        output='screen'
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 5. [필수] ROS 2 <-> Gazebo 브릿지 설정
    # 이게 없으면 Gazebo의 시간(Clock)이 ROS 2로 전달되지 않습니다.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 시뮬레이션 시간 동기화
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # LiDAR 데이터 (GZ 토픽 이름 'scan'이 URDF와 일치해야 함)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # TF (좌표 변환 데이터 - Gazebo에서 계산된 위치를 ROS로 가져옴)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Camera Data (Gazebo에서 생성된 카메라 데이터를 ROS로 가져옴)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # 6. [컨트롤러] ros2_control 컨트롤러 활성화
    # spawn_entity가 완료된 후에 실행되어야 합니다.
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_leg_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller'],
        output='screen',
    )

    # 7. [걸음걸이] Gait Controller 노드
    # cmd_vel → IK 계산 → 12개 관절 제어
    gait_node = Node(
        package='hyper_pupper_control',
        executable='gait_node',
        output='screen',
    )

    # 이벤트 핸들러: spawn 완료 후 → 컨트롤러 활성화 → gait 노드 시작
    activate_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_joint_state_broadcaster, spawn_leg_controller],
        )
    )

    start_gait_after_leg_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_leg_controller,
            on_exit=[gait_node],
        )
    )

    return LaunchDescription([
        set_gz_resource_path, # 환경 변수 설정을 리스트 맨 앞에 배치
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        activate_controllers_after_spawn,
        start_gait_after_leg_ctrl,
    ])