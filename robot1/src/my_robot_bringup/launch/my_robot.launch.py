#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    robot1_description_path = get_package_share_directory('robot1_description')
    
    # 重要：使用正确的文件名 - my_robot.urdf.xacro 而不是 robot1.urdf.xacro
    urdf_file = os.path.join(robot1_description_path, 'urdf', 'my_robot.urdf.xacro')
    
    print(f"Looking for URDF at: {urdf_file}")
    
    # 检查文件是否存在
    if not os.path.exists(urdf_file):
        print(f"ERROR: URDF file not found at {urdf_file}")
        # 尝试备选文件名
        alt_urdf_file = os.path.join(robot1_description_path, 'urdf', 'robot.urdf.xacro')
        if os.path.exists(alt_urdf_file):
            urdf_file = alt_urdf_file
            print(f"Using alternative: {urdf_file}")
        else:
            print("No URDF file found!")
            robot_desc = '<?xml version="1.0"?><robot name="error_robot"><link name="base_link"/></robot>'
            urdf_file = None
    
    # 执行 xacro 命令
    if urdf_file and os.path.exists(urdf_file):
        try:
            result = subprocess.run(
                ['xacro', urdf_file],
                capture_output=True,
                text=True,
                check=True
            )
            robot_desc = result.stdout
            print("Successfully parsed URDF with xacro")
            # 打印前200个字符检查
            print(f"URDF preview: {robot_desc[:200]}...")
        except subprocess.CalledProcessError as e:
            print(f"Error parsing URDF: {e.stderr}")
            robot_desc = ""
        except FileNotFoundError:
            print("xacro command not found. Installing...")
            os.system('sudo apt install -y ros-humble-xacro')
            robot_desc = ""
    else:
        robot_desc = '<?xml version="1.0"?><robot name="fallback_robot"><link name="base_link"/></robot>'
    
    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc
        }],
        output='screen'
    )
    
    # 关节状态发布器 GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # 控制器管理器配置文件
    bringup_path = get_package_share_directory('my_robot_bringup')
    controllers_file = os.path.join(bringup_path, 'config', 'ros2_controllers.yaml')
    
    # 控制器管理器
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[controllers_file],
        output='screen'
    )
    
    # 启动关节状态广播器
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster']
    )
    
    # 启动关节轨迹控制器
    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_trajectory_controller',
        arguments=['joint_trajectory_controller']
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller,
        rviz_node
    ])
