from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定位robot1_description功能包和servo.yaml配置文件
    pkg_robot1 = FindPackageShare(package='robot1_description').find('robot1_description')
    servo_config = PathJoinSubstitution([pkg_robot1, 'config', 'servo.yaml'])

    # 启动MoveIt Servo节点（关键：executable改为servo_node_main）
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',  # 匹配你系统中的可执行文件名
        name='servo_server',
        output='screen',
        parameters=[
            servo_config,          # 加载定制的servo.yaml
            {'use_sim_time': False} # 真实机器人设为false
        ]
    )

    return LaunchDescription([
        servo_node
    ])
