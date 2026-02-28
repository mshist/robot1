#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
# 注释掉不用的Command导入（可选）
# from launch.substitutions import Command  # 新增：读取SRDF

def generate_launch_description():
    # 1. 定位功能包路径（区分robot1_description和my_robot_bringup）
    robot1_desc_path = get_package_share_directory('robot1_description')
    bringup_path = get_package_share_directory('my_robot_bringup')
    
    # 2. 解析XACRO为URDF
    urdf_file = os.path.join(robot1_desc_path, 'urdf', 'my_robot.urdf.xacro')
    result = subprocess.run(['xacro', urdf_file], capture_output=True, text=True)
    robot_desc = result.stdout
    
    # 3. 读取SRDF文件（核心修改！直接读src目录的绝对路径）
    # 替换为你实际的SRDF文件路径（src目录）
    srdf_file = '/home/mshist/robot1/src/robot1_description/srdf/robot1.srdf'
    # 直接打开文件读取内容，不用cat命令
    with open(srdf_file, 'r', encoding='utf-8') as f:
        robot_desc_semantic = f.read()
    
    # 4. 控制器配置文件
    controllers_file = os.path.join(bringup_path, 'config', 'ros2_controllers.yaml')
    
    return LaunchDescription([
        # 机器人状态发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # ROS2控制节点
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_file, {'robot_description': robot_desc}]
        ),
        
        # 新增：MoveGroup节点（发布robot_description_semantic）
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'robot_description_semantic': robot_desc_semantic},
                {'use_sim_time': False},
                {'publish_robot_description_semantic': True},
                {'move_group_name': 'arm'}  # 匹配你的SRDF规划组
            ]
        ),
        
        # 控制器启动（保留原有延迟逻辑）
        TimerAction(
            period=3.0,
            actions=[Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])]
        ),
        TimerAction(
            period=4.0,
            actions=[Node(package='controller_manager', executable='spawner', arguments=['arm_controller'])]
        ),
        TimerAction(
            period=5.0,
            actions=[Node(package='controller_manager', executable='spawner', arguments=['gripper_controller'])]
        ),
        
        # RViz节点
        Node(package='rviz2', executable='rviz2'),
    ])
