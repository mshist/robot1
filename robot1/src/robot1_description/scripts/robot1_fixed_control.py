#!/usr/bin/env python3
"""
Robot1 超极简版：末端绝对位置控制（无语法错误）
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import sys
import select
import tty
import termios

# 基础配置
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
INIT_XYZ = [0.4, 0.0, 0.4]  # 初始末端绝对位置
STEP = 0.01  # 步长

# 极简逆运动学（保证关节联动）
def simple_ik(xyz):
    x, y, z = xyz
    # 直接映射：X/Y/Z变化→6个关节都动
    joint1 = np.arctan2(y, x) * 1.2
    joint2 = (z - 0.4) * 5 - 0.5
    joint3 = (x - 0.4) * 3 + 0.3
    joint4 = joint1 * 0.8
    joint5 = joint2 * 0.7
    joint6 = joint3 * 0.9
    # 关节限位
    joints = [joint1, joint2, joint3, joint4, joint5, joint6]
    limits = [(-np.pi, np.pi), (-1.57, 1.57), (-1.57, 1.57),
              (-np.pi, np.pi), (-1.57, 1.57), (-np.pi, np.pi)]
    for i in range(6):
        joints[i] = np.clip(joints[i], limits[i][0], limits[i][1])
    return joints

# 核心控制节点
class SimpleControl(Node):
    def __init__(self):
        super().__init__('robot1_simple_control')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.current_xyz = INIT_XYZ.copy()
        self.last_key = ''
        self.print_help()
        self.print_state()

    def print_help(self):
        self.get_logger().info("="*60)
        self.get_logger().info("Robot1 极简版控制（无语法错误）")
        self.get_logger().info("W/S: X±0.01 | A/D: Y±0.01 | Q/E: Z±0.01")
        self.get_logger().info("空格：发送指令 | H：重置 | Ctrl+C：退出")
        self.get_logger().info("="*60)

    def print_state(self):
        x, y, z = self.current_xyz
        joints = simple_ik(self.current_xyz)
        self.get_logger().info(f"📌 末端绝对位置：X={x:.3f} Y={y:.3f} Z={z:.3f}")
        self.get_logger().info(f"🔧 关节角度：j1={joints[0]:.3f} j2={joints[1]:.3f} j3={joints[2]:.3f} j4={joints[3]:.3f} j5={joints[4]:.3f} j6={joints[5]:.3f}")

    def send_command(self):
        joints = simple_ik(self.current_xyz)
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)
        self.get_logger().info("✅ 指令发送成功！")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            key = sys.stdin.read(1) if rlist else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def main():
    rclpy.init()
    node = SimpleControl()
    try:
        while rclpy.ok():
            key = node.get_key()
            if key == '': continue
            if key == node.last_key: continue
            node.last_key = key

            if key == '\x03': break
            elif key == ' ':
                node.send_command()
                node.print_state()
            elif key == 'h':
                node.current_xyz = INIT_XYZ.copy()
                node.get_logger().info("🔄 重置初始位置！")
                node.print_state()
            # 位置控制
            elif key == 'w': node.current_xyz[0] += STEP
            elif key == 's': node.current_xyz[0] -= STEP
            elif key == 'a': node.current_xyz[1] += STEP
            elif key == 'd': node.current_xyz[1] -= STEP
            elif key == 'q': node.current_xyz[2] += STEP
            elif key == 'e': node.current_xyz[2] -= STEP

            if key not in ['\x03', '']:
                node.print_state()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

