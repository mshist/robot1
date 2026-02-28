#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import tty
import termios

# 适配你的SRDF关节名（arm组的7个关节，含virtual/tool_joint则注释）
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# 初始关节角度（SRDF的home位姿）
INIT_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 步长（每次按键调整0.05rad≈2.86°）
STEP = 0.05

class Robot1JointControl(Node):
    def __init__(self):
        super().__init__('robot1_joint_control')
        # 发布到arm_controller的关节轨迹话题
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.current_joints = INIT_JOINTS.copy()
        self.print_help()

    def print_help(self):
        self.get_logger().info("="*50)
        self.get_logger().info("Robot1 关节控制（直接控制ROS2控制器）")
        self.get_logger().info("1-6键：对应关节1~6 +{:.1f}°".format(STEP*57.3))
        self.get_logger().info("!@#$%^：对应关节1~6 -{:.1f}°（Shift+1-6）".format(STEP*57.3))
        self.get_logger().info("H键：回到home位姿 | 空格：发送目标 | Ctrl+C：退出")
        self.get_logger().info("="*50)

    def send_trajectory(self):
        """发布关节轨迹指令"""
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = self.current_joints
        point.time_from_start.sec = 1  # 1秒内到达目标
        traj.points.append(point)
        self.traj_pub.publish(traj)
        self.get_logger().info(f"已发送目标关节角度：{[round(x,3) for x in self.current_joints]} rad")

    def get_key(self):
        """非阻塞读取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

def main(args=None):
    rclpy.init(args=args)
    node = Robot1JointControl()
    try:
        while rclpy.ok():
            key = node.get_key()
            if key == '\x03':  # Ctrl+C
                break
            elif key == ' ':  # 空格：发送目标
                node.send_trajectory()
            elif key == 'h' or key == 'H':  # H键：回到home
                node.current_joints = INIT_JOINTS.copy()
                node.get_logger().info("已重置为home位姿：{}".format(INIT_JOINTS))
            elif key in ['1','2','3','4','5','6']:  # 关节+
                idx = int(key)-1
                node.current_joints[idx] += STEP
                node.get_logger().info(f"关节{idx+1}：{node.current_joints[idx]:.3f} rad")
            elif key in ['!','@','#','$','%','^']:  # 关节-
                idx = ord(key)-ord('!')
                node.current_joints[idx] -= STEP
                node.get_logger().info(f"关节{idx+1}：{node.current_joints[idx]:.3f} rad")
    except Exception as e:
        node.get_logger().error(f"出错：{e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

