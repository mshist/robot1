#!/usr/bin/env python3
"""
Robot1 高速持续控制版：更快的运动速度，按住就快速运动
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import sys
import select
import tty
import termios
import time

# 高速配置（核心提速！）
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
INIT_XYZ = [0.0, 0.0, 0.4]
STEP = 0.04  # 高速步长
SEND_FREQ = 50  # 高速发送频率（每秒50次）
KEY_HOLD_INTERVAL = 0.01 # 更快的检测间隔（10ms）

# 按键状态
key_states = {'w':False,'s':False,'a':False,'d':False,'q':False,'e':False,'h':False}

# 逆运动学（不变）
def simple_ik(xyz):
    x, y, z = xyz
    joint1 = np.arctan2(y, x) * 1.2
    joint2 = (z - 0.4) * 5 - 0.5
    joint3 = (x - 0.4) * 3 + 0.3
    joint4 = joint1 * 0.8
    joint5 = joint2 * 0.7
    joint6 = joint3 * 0.9
    limits = [(-np.pi, np.pi), (-1.57, 1.57), (-1.57, 1.57),
              (-np.pi, np.pi), (-1.57, 1.57), (-np.pi, np.pi)]
    joints = [joint1, joint2, joint3, joint4, joint5, joint6]
    for i in range(6):
        joints[i] = np.clip(joints[i], limits[i][0], limits[i][1])
    return joints

# 非阻塞读键
def read_key_non_blocking():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    key = ''
    try:
        tty.setraw(fd)
        if select.select([sys.stdin], [], [], 0.001)[0]:
            key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# 核心控制节点
class HighSpeedControl(Node):
    def __init__(self):
        super().__init__('robot1_highspeed_control')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 30)
        self.current_xyz = INIT_XYZ.copy()
        self.last_send_time = time.time()
        self.print_help()
        self.print_state()

    def print_help(self):
        self.get_logger().info("="*70)
        self.get_logger().info("Robot1 高速持续控制版（按住快速运动）")
        self.get_logger().info("按住 W/S：X轴快速前后 | 按住 A/D：Y轴快速左右")
        self.get_logger().info("按住 Q/E：Z轴快速上下 | H：重置 | Ctrl+C：退出")
        self.get_logger().info("✅ 速度提升6倍，按住就快速运动！")
        self.get_logger().info("="*70)

    def print_state(self):
        x, y, z = self.current_xyz
        joints = simple_ik(self.current_xyz)
        self.get_logger().info(f"🚀 高速位置：X={x:.3f} Y={y:.3f} Z={z:.3f} | 关节j1={joints[0]:.3f} j2={joints[1]:.3f}")

    def send_highspeed_command(self):
        current_time = time.time()
        if current_time - self.last_send_time < 1/SEND_FREQ:
            return
        self.last_send_time = current_time
        
        joints = simple_ik(self.current_xyz)
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 100ms快速到达
        traj.points.append(point)
        self.pub.publish(traj)

    def update_position(self):
        global key_states
        if key_states['w']: self.current_xyz[0] += STEP
        if key_states['s']: self.current_xyz[0] -= STEP
        if key_states['a']: self.current_xyz[1] += STEP
        if key_states['d']: self.current_xyz[1] -= STEP
        if key_states['q']: self.current_xyz[2] += STEP
        if key_states['e']: self.current_xyz[2] -= STEP
        if key_states['h']:
            self.current_xyz = INIT_XYZ.copy()
            self.get_logger().info("🔄 重置初始位置！")
            key_states['h'] = False

def main():
    rclpy.init()
    node = HighSpeedControl()
    global key_states

    try:
        while rclpy.ok():
            key = read_key_non_blocking()
            if key == '\x03': break
            elif key != '':
                if key in key_states: key_states[key] = True
            else:
                for k in key_states: key_states[k] = False

            node.update_position()
            node.send_highspeed_command()

            if time.time() - node.last_send_time < 0.05:
                node.print_state()

            time.sleep(KEY_HOLD_INTERVAL)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

