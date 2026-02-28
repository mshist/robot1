#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Gazebo 启动文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('robot1_description'),
                'worlds',
                'empty.world'
            ]),
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )
    
    # URDF 文件路径
    urdf_file = PathJoinSubstitution([
        FindPackageShare('robot1_description'),
        'urdf',
        'robot1.gazebo.xacro'
    ])
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # Gazebo 中生成机器人
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot1',
        arguments=[
            '-entity', 'robot1',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot1_description'),
                'config',
                'controllers.yaml'
            ]),
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    # 启动控制器
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        controller_manager,
        joint_state_broadcaster,
        joint_trajectory_controller
    ])