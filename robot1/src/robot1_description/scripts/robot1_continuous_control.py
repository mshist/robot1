#!/usr/bin/env python3
"""
Robot1 实时连续控制版：按住按键持续调整末端位置，自动发送指令
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

# 基础配置
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
INIT_XYZ = [0.4, 0.0, 0.4]  # 初始末端绝对位置
STEP = 0.005  # 单次步长（更小，连续控制更丝滑）
SEND_FREQ = 10  # 自动发送频率（每秒10次）
KEY_HOLD_INTERVAL = 0.05  # 按键按住检测间隔（50ms）

# 全局按键状态（记录哪些按键被按住）
key_states = {
    'w': False, 's': False, 'a': False, 'd': False,
    'q': False, 'e': False, 'h': False
}

# 极简逆运动学（保证关节联动）
def simple_ik(xyz):
    x, y, z = xyz
    joint1 = np.arctan2(y, x) * 1.2
    joint2 = (z - 0.4) * 5 - 0.5
    joint3 = (x - 0.4) * 3 + 0.3
    joint4 = joint1 * 0.8
    joint5 = joint2 * 0.7
    joint6 = joint3 * 0.9
    # 关节限位
    limits = [(-np.pi, np.pi), (-1.57, 1.57), (-1.57, 1.57),
              (-np.pi, np.pi), (-1.57, 1.57), (-np.pi, np.pi)]
    joints = [joint1, joint2, joint3, joint4, joint5, joint6]
    for i in range(6):
        joints[i] = np.clip(joints[i], limits[i][0], limits[i][1])
    return joints

# 非阻塞按键读取（检测按住/松开）
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

# 核心连续控制节点
class ContinuousControl(Node):
    def __init__(self):
        super().__init__('robot1_continuous_control')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 20)
        self.current_xyz = INIT_XYZ.copy()
        self.last_send_time = time.time()
        self.print_help()
        self.print_state()

    def print_help(self):
        self.get_logger().info("="*70)
        self.get_logger().info("Robot1 实时连续控制版（按住按键持续运动）")
        self.get_logger().info("按住 W/S：X轴连续前后 | 按住 A/D：Y轴连续左右")
        self.get_logger().info("按住 Q/E：Z轴连续上下 | H：重置位置 | Ctrl+C：退出")
        self.get_logger().info("✅ 按住按键自动连续发送指令，松开即停！")
        self.get_logger().info("="*70)

    def print_state(self):
        x, y, z = self.current_xyz
        joints = simple_ik(self.current_xyz)
        self.get_logger().info(f"📌 实时末端位置：X={x:.3f} Y={y:.3f} Z={z:.3f} | 关节j1={joints[0]:.3f} j2={joints[1]:.3f} j3={joints[2]:.3f}")

    def send_continuous_command(self):
        """自动按频率发送指令"""
        current_time = time.time()
        # 按设定频率发送（避免刷屏）
        if current_time - self.last_send_time < 1/SEND_FREQ:
            return
        self.last_send_time = current_time
        
        joints = simple_ik(self.current_xyz)
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 0  # 实时指令，立即执行
        point.time_from_start.nanosec = 200000000  # 200ms到达
        traj.points.append(point)
        self.pub.publish(traj)

    def update_position(self):
        """根据按住的按键更新位置"""
        global key_states
        if key_states['w']:
            self.current_xyz[0] += STEP
        if key_states['s']:
            self.current_xyz[0] -= STEP
        if key_states['a']:
            self.current_xyz[1] += STEP
        if key_states['d']:
            self.current_xyz[1] -= STEP
        if key_states['q']:
            self.current_xyz[2] += STEP
        if key_states['e']:
            self.current_xyz[2] -= STEP
        if key_states['h']:
            self.current_xyz = INIT_XYZ.copy()
            self.get_logger().info("🔄 重置为初始位置！")
            key_states['h'] = False  # 重置后清空H键状态

def main():
    rclpy.init()
    node = ContinuousControl()
    global key_states

    try:
        while rclpy.ok():
            # 1. 读取按键（检测按住/松开）
            key = read_key_non_blocking()
            if key == '\x03':  # Ctrl+C退出
                break
            elif key != '':
                # 按下按键：标记为True
                if key in key_states:
                    key_states[key] = True
            else:
                # 无按键：标记所有按键为False（松开）
                for k in key_states:
                    key_states[k] = False

            # 2. 更新末端位置（连续调整）
            node.update_position()

            # 3. 自动发送指令（连续控制）
            node.send_continuous_command()

            # 4. 定期打印状态（每秒1次）
            if time.time() - node.last_send_time < 0.1:
                node.print_state()

            # 5. 控制循环频率
            time.sleep(KEY_HOLD_INTERVAL)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


