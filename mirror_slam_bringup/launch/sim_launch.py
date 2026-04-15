"""One-shot simulation launch: TB3 + Nav2/SLAM + UR3e (tf_prefix=arm_) + MoveIt + probe + explorer.

The UR driver is launched with tf_prefix:=arm_ so all UR frames become
arm_base_link, arm_shoulder_link, etc.  arm_ur_moveit_config has matching
prefixed joint/link names.  No TF collision with the TurtleBot3.

Equivalent manual terminals:

  T1: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e \\
        use_mock_hardware:=true robot_ip:=0.0.0.0 launch_rviz:=false tf_prefix:=arm_
  T2: ros2 launch arm_ur_moveit_config arm_ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
  T3: ros2 launch nav2_bringup tb3_simulation_launch.py world:=... params_file:=... slam:=True
  T4: ros2 run ur3 arm_probe_server --ros-args -p use_sim_time:=true
  T5: ros2 run gap_explorer gap_explorer --ros-args -p use_sim_time:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Top-level arg declarations ────────────────────────────────────
    # OnProcessExit handlers inside included launch files resolve
    # LaunchConfigurations in the PARENT scope.  Every arg that
    # ur_control.launch.py or arm_ur_moveit.launch.py might reference
    # must be declared here.
    arg_declarations = [
        # ur_control.launch.py args
        DeclareLaunchArgument('ur_type',           default_value='ur3e'),
        DeclareLaunchArgument('use_mock_hardware',  default_value='true'),
        DeclareLaunchArgument('robot_ip',           default_value='0.0.0.0'),
        DeclareLaunchArgument('tf_prefix',          default_value='arm_'),
        # arm_ur_moveit.launch.py args
        DeclareLaunchArgument('launch_rviz',        default_value='true'),
        DeclareLaunchArgument('warehouse_sqlite_path',
                              default_value=os.path.expanduser(
                                  '~/.ros/warehouse_ros.sqlite')),
        DeclareLaunchArgument('launch_servo',       default_value='false'),
        DeclareLaunchArgument('use_sim_time',       default_value='false'),
        DeclareLaunchArgument('publish_robot_description_semantic',
                              default_value='true'),
    ]

    # ── Paths ──────────────────────────────────────────────────────────
    world_file = os.path.join(
        get_package_share_directory('mirror_slam_bringup'),
        'worlds', 'simpleroom.sdf.xacro',
    )
    nav2_params = os.path.join(
        get_package_share_directory('mirror_slam_bringup'),
        'params', 'nav2_params.yaml',
    )
    gap_explorer_params = os.path.join(
        get_package_share_directory('gap_explorer'),
        'config', 'gap_explorer.yaml',
    )

    # ── 1. UR3e driver (tf_prefix=arm_) ───────────────────────────────
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'), 'launch',
                'ur_control.launch.py',
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur3e',
            'robot_ip': '0.0.0.0',
            'use_mock_hardware': 'true',
            'launch_rviz': 'false',
            'tf_prefix': 'arm_',
        }.items(),
    )

    # ── 2. MoveIt (arm_ur_moveit_config, delayed) ─────────────────────
    moveit = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('arm_ur_moveit_config'), 'launch',
                        'arm_ur_moveit.launch.py',
                    ])
                ]),
                launch_arguments={
                    'ur_type': 'ur3e',
                    'launch_rviz': 'true',
                }.items(),
            ),
        ],
    )

    # ── 3. TB3 sim + Nav2 + SLAM (delayed) ────────────────────────────
    tb3_sim = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'), 'launch',
                        'tb3_simulation_launch.py',
                    ])
                ]),
                launch_arguments={
                    'world': world_file,
                    'params_file': nav2_params,
                    'x_pose': '2.0',
                    'y_pose': '0.0',
                    'z_pose': '0.1',
                    'slam': 'True',
                    'headless': 'False',
                }.items(),
            ),
        ],
    )

    # ── 4. Arm probe action server ────────────────────────────────────
    arm_probe = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='ur3',
                executable='arm_probe_server',
                name='arm_probe_server',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # ── 5. Gap explorer ──────────────────────────────────────────────
    gap_explorer = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='gap_explorer',
                executable='gap_explorer',
                name='gap_explorer',
                output='screen',
                parameters=[gap_explorer_params, {'use_sim_time': True}],
            ),
        ],
    )

    return LaunchDescription(
        arg_declarations + [
            ur_driver,
            moveit,
            tb3_sim,
            arm_probe,
            gap_explorer,
        ]
    )
