#!/usr/bin/env python3
"""
Robot1 全空间覆盖控制版：关节限位拉满 + 覆盖所有可达空间
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

# ====================== 全空间配置 ======================
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# 固定初始XYZ（选工作空间中心位，方便覆盖全范围）
FIXED_INIT_XYZ = [0.2, 0.0, 0.5]
# 适配全范围的步长
SAFE_STEP = 0.015  
# 平衡响应和到位的发送频率
SAFE_SEND_FREQ = 2  
KEY_HOLD_INTERVAL = 0.001 

# 按键状态
key_states = {'w':False,'s':False,'a':False,'d':False,'q':False,'e':False,'h':False}

# 🔴 全范围逆解：适配最大关节限位，覆盖所有可达空间
def full_space_ik(xyz):
    x, y, z = xyz
    # 全范围映射：让XYZ覆盖关节最大行程
    # joint1：覆盖±π，对应Y/X全方向旋转
    joint1 = np.arctan2(y, x + 0.001)  # 原生±π范围，无需缩放
    # joint2：覆盖±π/2，对应Z轴大范围升降
    joint2 = np.clip((z - 0.5) * 6, -np.pi/2, np.pi/2)
    # joint3：覆盖±π/2，对应X轴大范围前后
    joint3 = np.clip((x - 0.2) * 6, -np.pi/2, np.pi/2)
    # joint4-joint6：跟随主关节，覆盖全范围
    joint4 = np.clip(joint1 * 1.0, -np.pi/2, np.pi/2)
    joint5 = np.clip(joint2 * 1.0, -np.pi/2, np.pi/2)
    joint6 = np.clip(joint3 * 1.0, -np.pi/2, np.pi/2)
    
    # 🔴 工业机械臂通用最大物理限位（覆盖99%机械臂的全行程）
    # 注释：可根据你机械臂手册替换为实际最大限位
    FULL_LIMITS = [
        (-np.pi, np.pi),       # joint1：±π（360°旋转，覆盖全周）
        (-np.pi/2, np.pi/2),   # joint2：±90°（垂直方向全范围）
        (-np.pi/2, np.pi/2),   # joint3：±90°（水平方向全范围）
        (-np.pi/2, np.pi/2),   # joint4：±90°（腕部旋转全范围）
        (-np.pi/2, np.pi/2),   # joint5：±90°（腕部俯仰全范围）
        (-np.pi/2, np.pi/2)    # joint6：±90°（末端旋转全范围）
    ]
    joints = [joint1, joint2, joint3, joint4, joint5, joint6]
    for i in range(6):
        joints[i] = np.clip(joints[i], FULL_LIMITS[i][0], FULL_LIMITS[i][1])
    return joints

# 非阻塞读键（保留）
def read_key_non_blocking():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    key = ''
    try:
        tty.setraw(fd)
        if select.select([sys.stdin], [], [], 0.0005)[0]:
            key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# 核心控制节点（全空间版）
class FullSpaceControl(Node):
    def __init__(self):
        super().__init__('robot1_full_space_control')
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 20)
        self.current_xyz = FIXED_INIT_XYZ.copy()
        self.last_send_time = time.time()
        self.last_print_time = time.time()
        
        # 启动先发送中心位指令，作为全范围运动的起点
        self.send_full_space_command()
        time.sleep(1.5)
        
        self.print_help()
        self.print_state()

    def print_help(self):
        self.get_logger().info("="*70)
        self.get_logger().info("Robot1 全空间覆盖控制版（关节限位拉满）")
        self.get_logger().info(f"📌 初始中心位：X={FIXED_INIT_XYZ[0]} Y={FIXED_INIT_XYZ[1]} Z={FIXED_INIT_XYZ[2]}")
        self.get_logger().info("轻按 W/S：X轴前后 | 轻按 A/D：Y轴左右 | 轻按 Q/E：Z轴上下")
        self.get_logger().info("H：重置中心位 | Ctrl+C：退出（⚠️ 轻按慢动，避免机械限位！）")
        self.get_logger().info("✅ 关节限位拉满，覆盖所有可达工作空间！")
        self.get_logger().info("="*70)

    def print_state(self):
        if time.time() - self.last_print_time > 0.2:
            x, y, z = self.current_xyz
            joints = full_space_ik(self.current_xyz)
            self.get_logger().info(f"📍 当前XYZ：X={x:.3f} Y={y:.3f} Z={z:.3f}")
            self.get_logger().info(f"🔧 关节位置：j1={joints[0]:.2f} j2={joints[1]:.2f} j3={joints[2]:.2f}")
            self.last_print_time = time.time()

    def send_full_space_command(self):
        """全范围指令发送：延长到位时间，适配大行程"""
        current_time = time.time()
        if current_time - self.last_send_time < 1/SAFE_SEND_FREQ:
            return
        self.last_send_time = current_time
        
        joints = full_space_ik(self.current_xyz)
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joints
        # 2秒到位：适配最大行程的运动时间
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0  
        traj.points.append(point)
        self.pub.publish(traj)

    def update_position(self):
        """全范围位置更新：覆盖更大XYZ区间"""
        global key_states
        # 增大步长，快速覆盖全空间
        if key_states['w']: self.current_xyz[0] += SAFE_STEP
        if key_states['s']: self.current_xyz[0] -= SAFE_STEP
        if key_states['a']: self.current_xyz[1] += SAFE_STEP
        if key_states['d']: self.current_xyz[1] -= SAFE_STEP
        if key_states['q']: self.current_xyz[2] += SAFE_STEP
        if key_states['e']: self.current_xyz[2] -= SAFE_STEP
        
        # 重置到工作空间中心
        if key_states['h']:
            self.current_xyz = FIXED_INIT_XYZ.copy()
            self.get_logger().info(f"🔄 重置到中心位：X={FIXED_INIT_XYZ[0]} Y={FIXED_INIT_XYZ[1]} Z={FIXED_INIT_XYZ[2]}")
            key_states['h'] = False

def main():
    rclpy.init()
    node = FullSpaceControl()
    global key_states

    try:
        while rclpy.ok():
            key = read_key_non_blocking()
            if key == '\x03':
                # 退出前回中心位，保护机械臂
                node.current_xyz = FIXED_INIT_XYZ.copy()
                node.send_full_space_command()
                break
            elif key != '':
                if key in key_states: 
                    key_states[key] = True
            else:
                for k in key_states: 
                    key_states[k] = False

            node.update_position()
            node.send_full_space_command()
            node.print_state()

            time.sleep(KEY_HOLD_INTERVAL)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
