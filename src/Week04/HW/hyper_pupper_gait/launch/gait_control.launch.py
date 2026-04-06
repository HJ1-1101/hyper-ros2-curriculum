"""
Launch file for hyper_pupper_gait.

Starts two nodes:
  1. gait_controller_node  — /cmd_vel → IK → /leg_controller/follow_trajectory
  2. teleop_key_node       — keyboard → /cmd_vel   (in a new xterm window)

Usage
-----
Default settings:
    ros2 launch hyper_pupper_gait gait_control.launch.py

Custom gait parameters:
    ros2 launch hyper_pupper_gait gait_control.launch.py body_height:=0.08 gait_period:=0.4

Arguments
---------
  body_height   float   Base_link height above ground [m]   (default 0.07)
  gait_period   float   Stride duration [s]                 (default 0.50)
  step_height   float   Peak swing height [m]               (default 0.03)
  duty_factor   float   Fraction of cycle in stance         (default 0.50)
  ctrl_rate     float   Gait controller update rate [Hz]    (default 50.0)

Student exercise
----------------
Before running this launch file, ensure the simulation controllers are active:

    # Terminal 1 — launch Gazebo
    ros2 launch hyper_pupper_complete gazebo.launch.py

    # Terminal 2 — activate ros2_control controllers
    ros2 control load_controller --set-state active joint_state_broadcaster
    ros2 control load_controller --set-state active leg_controller

    # Terminal 3 — start gait + teleop
    ros2 launch hyper_pupper_gait gait_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch arguments ----
    args = [
        DeclareLaunchArgument(
            'body_height', default_value='0.07',
            description='Base_link height above ground [m]'),
        DeclareLaunchArgument(
            'gait_period', default_value='0.50',
            description='Full stride duration [s]'),
        DeclareLaunchArgument(
            'step_height', default_value='0.03',
            description='Peak foot lift during swing [m]'),
        DeclareLaunchArgument(
            'duty_factor', default_value='0.50',
            description='Fraction of each cycle spent in stance (0–1)'),
        DeclareLaunchArgument(
            'ctrl_rate', default_value='50.0',
            description='Gait controller update rate [Hz]'),
    ]

    # ---- Gait controller node ----
    gait_controller = Node(
        package='hyper_pupper_gait',
        executable='gait_controller_node',
        name='gait_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'body_height': LaunchConfiguration('body_height'),
            'gait_period': LaunchConfiguration('gait_period'),
            'step_height': LaunchConfiguration('step_height'),
            'duty_factor': LaunchConfiguration('duty_factor'),
            'ctrl_rate':   LaunchConfiguration('ctrl_rate'),
        }],
    )

    # ---- Teleop keyboard node ----
    # Launched in a new xterm so the raw TTY input doesn't conflict with
    # the launch terminal.  Remove prefix='xterm -e' if running inside
    # a terminal multiplexer or a headless environment, and run the node
    # separately: ros2 run hyper_pupper_gait teleop_key_node
    teleop = Node(
        package='hyper_pupper_gait',
        executable='teleop_key_node',
        name='teleop_key',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription(args + [gait_controller, teleop])
