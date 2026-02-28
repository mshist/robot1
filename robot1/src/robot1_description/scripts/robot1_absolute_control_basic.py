#!/usr/bin/env python3
"""
Robot1 进阶版末端绝对控制（支持无限次连续控制）
修复：解决“只能控制一次”问题，支持连续按键/发送指令
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import select
import tty
import termios
import time

# ========== 核心参数（保持不变）==========
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
DH_PARAMS = {
    'a': [0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
    'alpha': [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0],
    'd': [0.1, 0.0, 0.0, 0.3, 0.0, 0.1],
    'theta0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
INIT_POSE = [0.4, 0.0, 0.4, 0.0, np.pi/2, 0.0]
POS_STEP = 0.01
ROT_STEP = 0.05
JOINT_LIMITS = [
    (-np.pi, np.pi), (-1.57, 1.57), (-1.57, 1.57),
    (-np.pi, np.pi), (-1.57, 1.57), (-np.pi, np.pi)
]
WORKSPACE_LIMITS = {'x': (0.2, 0.6), 'y': (-0.3, 0.3), 'z': (0.2, 0.8)}

# ========== 逆运动学求解器（保持不变）==========
class IKSolver:
    def __init__(self, dh_params, joint_limits):
        self.dh_params = dh_params
        self.joint_limits = joint_limits
        self.last_joints = np.zeros(6)

    def dh_transform(self, theta, a, alpha, d):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,             np.sin(alpha),              np.cos(alpha),               d],
            [0,             0,                          0,                           1]
        ])

    def forward_kinematics(self, joints):
        T = np.eye(4)
        for i in range(6):
            theta = joints[i] + self.dh_params['theta0'][i]
            a = self.dh_params['a'][i]
            alpha = self.dh_params['alpha'][i]
            d = self.dh_params['d'][i]
            T_i = self.dh_transform(theta, a, alpha, d)
            T = T @ T_i
        return T

    def pose_to_matrix(self, xyzrpy):
        x, y, z, rx, ry, rz = xyzrpy
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        rot = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
        T[:3, :3] = rot
        return T

    def ik_6dof(self, target_xyzrpy):
        target_T = self.pose_to_matrix(target_xyzrpy)
        def cost_func(joints):
            current_T = self.forward_kinematics(joints)
            pos_error = np.linalg.norm(current_T[:3, 3] - target_T[:3, 3])
            rot_error = np.linalg.norm(current_T[:3, :3] - target_T[:3, :3])
            return pos_error + 0.5 * rot_error
        from scipy.optimize import minimize
        result = minimize(
            cost_func, self.last_joints, bounds=self.joint_limits,
            method='L-BFGS-B', options={'maxiter': 1000, 'gtol': 1e-6}
        )
        if result.success:
            self.last_joints = result.x
        return self.last_joints

# ========== 修复后的核心控制节点 ==========
class AdvancedAbsoluteControl(Node):
    def __init__(self):
        super().__init__('robot1_advanced_control')
        # 1. 发布器（增加队列大小，防止丢包）
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 50)
        self.pose_pub = self.create_publisher(PoseStamped, '/end_effector_target_pose', 50)
        # 2. 逆运动学求解器
        self.ik_solver = IKSolver(DH_PARAMS, JOINT_LIMITS)
        # 3. 当前末端位姿（用列表更易修改）
        self.current_pose = list(INIT_POSE)
        # 4. 终端设置（修复键盘读取）
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        # 5. 打印帮助
        self.print_help()
        self.print_current_state()

    def print_help(self):
        self.get_logger().info("="*80)
        self.get_logger().info("Robot1 进阶版末端控制（支持连续控制）")
        self.get_logger().info("位置：W/S(X) A/D(Y) Q/E(Z) | 姿态：U/J(RX) I/K(RY) O/L(RZ)")
        self.get_logger().info("空格：发送指令 | H：重置 | Ctrl+C：退出")
        self.get_logger().info("="*80)

    def check_workspace(self):
        """检查并限制工作空间"""
        x, y, z = self.current_pose[:3]
        self.current_pose[0] = np.clip(x, WORKSPACE_LIMITS['x'][0], WORKSPACE_LIMITS['x'][1])
        self.current_pose[1] = np.clip(y, WORKSPACE_LIMITS['y'][0], WORKSPACE_LIMITS['y'][1])
        self.current_pose[2] = np.clip(z, WORKSPACE_LIMITS['z'][0], WORKSPACE_LIMITS['z'][1])

    def print_current_state(self):
        """实时打印状态"""
        self.check_workspace()
        current_joints = self.ik_solver.ik_6dof(self.current_pose)
        x, y, z, rx, ry, rz = self.current_pose
        self.get_logger().info(f"📌 当前目标：X={x:.3f} Y={y:.3f} Z={z:.3f} | RX={rx:.3f} RY={ry:.3f} RZ={rz:.3f}")
        self.get_logger().info(f"🔧 关节角度：j1={current_joints[0]:.3f} j2={current_joints[1]:.3f} j3={current_joints[2]:.3f} j4={current_joints[3]:.3f} j5={current_joints[4]:.3f} j6={current_joints[5]:.3f}")

    def send_advanced_command(self):
        """发送指令（增加延迟，确保控制器接收）"""
        target_joints = self.ik_solver.ik_6dof(self.current_pose)
        # 构造轨迹消息
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = list(target_joints)
        point.time_from_start.sec = 1
        traj.points.append(point)
        # 发布指令（重复发布2次，防止丢包）
        self.traj_pub.publish(traj)
        time.sleep(0.1)
        self.traj_pub.publish(traj)
        # 发布可视化位姿
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.current_pose[0]
        pose_msg.pose.position.y = self.current_pose[1]
        pose_msg.pose.position.z = self.current_pose[2]
        rot = R.from_euler('xyz', self.current_pose[3:]).as_quat()
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]
        self.pose_pub.publish(pose_msg)
        # 打印反馈
        self.get_logger().info("✅ 指令发送成功！机械臂正在运动...")

    def get_key_continuous(self):
        """修复：持续监听键盘输入（阻塞式，确保响应）"""
        tty.setraw(self.fd)
        key = sys.stdin.read(1)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return key

    def cleanup(self):
        """退出时恢复终端设置"""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        self.get_logger().info("👋 程序已退出，终端设置已恢复")

def main():
    rclpy.init()
    node = AdvancedAbsoluteControl()
    try:
        # 核心修复：无限循环+持续监听键盘
        while rclpy.ok():
            # 保持节点自旋，确保ROS2通信正常
            rclpy.spin_once(node, timeout_sec=0.01)
            # 读取键盘输入（阻塞式，确保每次按键都响应）
            key = node.get_key_continuous()
            
            if key == '\x03':  # Ctrl+C退出
                break
            elif key == ' ':   # 空格发送指令
                node.send_advanced_command()
                # 发送后刷新状态
                node.print_current_state()
            elif key == 'h':   # H重置
                node.current_pose = list(INIT_POSE)
                node.get_logger().info("🔄 已重置为初始位姿！")
                node.print_current_state()
            # 位置控制
            elif key == 'w': node.current_pose[0] += POS_STEP
            elif key == 's': node.current_pose[0] -= POS_STEP
            elif key == 'a': node.current_pose[1] += POS_STEP
            elif key == 'd': node.current_pose[1] -= POS_STEP
            elif key == 'q': node.current_pose[2] += POS_STEP
            elif key == 'e': node.current_pose[2] -= POS_STEP
            # 姿态控制
            elif key == 'u': node.current_pose[3] += ROT_STEP
            elif key == 'j': node.current_pose[3] -= ROT_STEP
            elif key == 'i': node.current_pose[4] += ROT_STEP
            elif key == 'k': node.current_pose[4] -= ROT_STEP
            elif key == 'o': node.current_pose[5] += ROT_STEP
            elif key == 'l': node.current_pose[5] -= ROT_STEP
            
            # 任意按键后刷新状态（除了退出键）
            if key not in ['\x03', '']:
                node.print_current_state()
                # 恢复终端设置，避免乱码
                termios.tcsetattr(node.fd, termios.TCSADRAIN, node.old_settings)
    except Exception as e:
        node.get_logger().error(f"❌ 程序出错：{str(e)}")
    finally:
        # 退出时清理
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
