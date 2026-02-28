#!/usr/bin/env python3
"""
Robot1 进阶版末端绝对位置+姿态控制（6轴全联动）
核心：X/Y/Z（位置）+ RX/RY/RZ（姿态）全维度绝对控制，6关节联动
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

# ========== 机械臂核心参数（务必根据你的URDF修改！）==========
# 1. 关节名（匹配你的SRDF/URDF）
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# 2. DH参数（6轴机械臂标准DH，从URDF提取）
DH_PARAMS = {
    'a': [0.0, 0.0, 0.3, 0.0, 0.0, 0.0],    # 连杆长度
    'alpha': [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0],  # 连杆扭转角
    'd': [0.1, 0.0, 0.0, 0.3, 0.0, 0.1],    # 连杆偏移
    'theta0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 关节初始角度
}
# 3. 初始末端绝对位姿（X/Y/Z/RX/RY/RZ，单位：m/rad）
INIT_POSE = [0.4, 0.0, 0.4, 0.0, np.pi/2, 0.0]
# 4. 调整步长（位置/姿态）
POS_STEP = 0.01    # 位置步长（米）
ROT_STEP = 0.05    # 姿态步长（弧度）
# 5. 关节限位（rad）
JOINT_LIMITS = [
    (-np.pi, np.pi),    # joint1
    (-1.57, 1.57),      # joint2
    (-1.57, 1.57),      # joint3
    (-np.pi, np.pi),    # joint4
    (-1.57, 1.57),      # joint5
    (-np.pi, np.pi)     # joint6
]
# 6. 工作空间边界（防止超限）
WORKSPACE_LIMITS = {
    'x': (0.2, 0.6),
    'y': (-0.3, 0.3),
    'z': (0.2, 0.8)
}

# ========== 进阶逆运动学求解器（6轴全联动）==========
class IKSolver:
    def __init__(self, dh_params, joint_limits):
        self.dh_params = dh_params
        self.joint_limits = joint_limits
        self.last_joints = np.zeros(6)  # 上一次关节角度（优化收敛速度）

    def dh_transform(self, theta, a, alpha, d):
        """标准DH变换矩阵"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,             np.sin(alpha),              np.cos(alpha),               d],
            [0,             0,                          0,                           1]
        ])

    def forward_kinematics(self, joints):
        """正运动学：关节角度 → 末端位姿矩阵"""
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
        """X/Y/Z/RX/RY/RZ → 齐次变换矩阵"""
        x, y, z, rx, ry, rz = xyzrpy
        # 位置部分
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        # 姿态部分（RPY→旋转矩阵）
        rot = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
        T[:3, :3] = rot
        return T

    def ik_6dof(self, target_xyzrpy):
        """6轴逆运动学：末端位姿 → 关节角度（数值解法+梯度下降）"""
        target_T = self.pose_to_matrix(target_xyzrpy)
        
        # 代价函数：末端位姿误差
        def cost_func(joints):
            current_T = self.forward_kinematics(joints)
            # 位置误差
            pos_error = np.linalg.norm(current_T[:3, 3] - target_T[:3, 3])
            # 姿态误差（旋转矩阵差值）
            rot_error = np.linalg.norm(current_T[:3, :3] - target_T[:3, :3])
            return pos_error + 0.5 * rot_error
        
        # 优化求解（用上次结果作为初始值，加快收敛）
        from scipy.optimize import minimize
        result = minimize(
            cost_func,
            self.last_joints,
            bounds=self.joint_limits,
            method='L-BFGS-B',
            options={'maxiter': 1000, 'gtol': 1e-6}
        )
        
        if result.success:
            self.last_joints = result.x  # 更新上次结果
            return result.x
        else:
            return self.last_joints  # 失败则返回上次值

# ========== 核心控制节点（进阶版）==========
class AdvancedAbsoluteControl(Node):
    def __init__(self):
        super().__init__('robot1_advanced_control')
        
        # 1. 创建发布器
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/end_effector_target_pose',
            10
        )
        
        # 2. 初始化逆运动学求解器
        self.ik_solver = IKSolver(DH_PARAMS, JOINT_LIMITS)
        
        # 3. 当前末端绝对位姿
        self.current_pose = np.array(INIT_POSE)
        
        # 4. 打印操作说明
        self.print_help()
        self.print_current_state()

    def print_help(self):
        self.get_logger().info("="*80)
        self.get_logger().info("Robot1 进阶版末端绝对位置+姿态控制（6轴全联动）")
        self.get_logger().info("┌─────────────────────────────────────────────────────┐")
        self.get_logger().info("│ 位置控制（X/Y/Z，单位：m）                          │")
        self.get_logger().info("│ W/S：X±0.01  |  A/D：Y±0.01  |  Q/E：Z±0.01         │")
        self.get_logger().info("│ 姿态控制（RX/RY/RZ，单位：rad）                      │")
        self.get_logger().info("│ U/J：RX±0.05 |  I/K：RY±0.05 |  O/L：RZ±0.05         │")
        self.get_logger().info("│ 功能键                                              │")
        self.get_logger().info("│ 空格：发送指令  |  H：重置初始位姿  |  Ctrl+C：退出 │")
        self.get_logger().info("└─────────────────────────────────────────────────────┘")
        self.get_logger().info("="*80)

    def check_workspace(self):
        """检查末端位置是否在工作空间内，超出则限制"""
        x, y, z = self.current_pose[:3]
        x = np.clip(x, WORKSPACE_LIMITS['x'][0], WORKSPACE_LIMITS['x'][1])
        y = np.clip(y, WORKSPACE_LIMITS['y'][0], WORKSPACE_LIMITS['y'][1])
        z = np.clip(z, WORKSPACE_LIMITS['z'][0], WORKSPACE_LIMITS['z'][1])
        self.current_pose[:3] = [x, y, z]
        return self.current_pose

    def print_current_state(self):
        """实时打印末端位姿+关节角度（进阶版反馈）"""
        # 检查工作空间
        self.check_workspace()
        # 求解当前关节角度
        current_joints = self.ik_solver.ik_6dof(self.current_pose)
        
        # 格式化输出
        x, y, z, rx, ry, rz = self.current_pose
        self.get_logger().info("📊 当前末端绝对状态：")
        self.get_logger().info(f"   位置：X={x:.3f} Y={y:.3f} Z={z:.3f} m")
        self.get_logger().info(f"   姿态：RX={rx:.3f} RY={ry:.3f} RZ={rz:.3f} rad")
        self.get_logger().info(f"   关节：j1={current_joints[0]:.3f} j2={current_joints[1]:.3f} j3={current_joints[2]:.3f}")
        self.get_logger().info(f"         j4={current_joints[3]:.3f} j5={current_joints[4]:.3f} j6={current_joints[5]:.3f}")

    def send_advanced_command(self):
        """发送进阶版绝对位姿指令"""
        # 1. 求解6轴关节角度
        target_joints = self.ik_solver.ik_6dof(self.current_pose)
        
        # 2. 构造轨迹消息
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = target_joints.tolist()
        point.time_from_start.sec = 1  # 1秒平滑到达
        point.time_from_start.nanosec = 500000000  # 0.5秒额外缓冲
        traj.points.append(point)
        
        # 3. 发布末端目标位姿（RViz可视化）
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.current_pose[0]
        pose_msg.pose.position.y = self.current_pose[1]
        pose_msg.pose.position.z = self.current_pose[2]
        # 姿态转为四元数
        rot = R.from_euler('xyz', self.current_pose[3:]).as_quat()
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]
        
        # 4. 发布指令
        self.traj_pub.publish(traj)
        self.pose_pub.publish(pose_msg)
        
        # 5. 打印执行反馈
        self.get_logger().info("✅ 已发送6轴联动绝对位姿指令！")
        self.print_current_state()

    def get_key(self):
        """非阻塞键盘读取"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            key = sys.stdin.read(1) if rlist else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def main():
    rclpy.init()
    node = AdvancedAbsoluteControl()
    
    try:
        while rclpy.ok():
            key = node.get_key()
            
            if key == '\x03':  # Ctrl+C退出
                node.get_logger().info("👋 退出进阶版控制程序")
                break
            
            elif key == ' ':   # 空格发送指令
                node.send_advanced_command()
            
            elif key == 'h':   # H重置初始位姿
                node.current_pose = np.array(INIT_POSE)
                node.get_logger().info("🔄 重置为初始末端位姿！")
                node.print_current_state()
            
            # ========== 位置控制 ==========
            elif key == 'w': node.current_pose[0] += POS_STEP  # X+
            elif key == 's': node.current_pose[0] -= POS_STEP  # X-
            elif key == 'a': node.current_pose[1] += POS_STEP  # Y+
            elif key == 'd': node.current_pose[1] -= POS_STEP  # Y-
            elif key == 'q': node.current_pose[2] += POS_STEP  # Z+
            elif key == 'e': node.current_pose[2] -= POS_STEP  # Z-
            
            # ========== 姿态控制 ==========
            elif key == 'u': node.current_pose[3] += ROT_STEP  # RX+
            elif key == 'j': node.current_pose[3] -= ROT_STEP  # RX-
            elif key == 'i': node.current_pose[4] += ROT_STEP  # RY+
            elif key == 'k': node.current_pose[4] -= ROT_STEP  # RY-
            elif key == 'o': node.current_pose[5] += ROT_STEP  # RZ+
            elif key == 'l': node.current_pose[5] -= ROT_STEP  # RZ-
            
            # 实时更新状态显示
            if key in ['w','s','a','d','q','e','u','j','i','k','o','l']:
                node.print_current_state()
    
    except Exception as e:
        node.get_logger().error(f"❌ 程序出错：{str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # 安装依赖（首次运行需执行）
    # pip3 install scipy
    main()

