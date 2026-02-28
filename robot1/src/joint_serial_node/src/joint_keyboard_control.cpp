#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <cmath>

// 关节轨迹动作类型定义
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class JointKeyboardControl : public rclcpp::Node {
public:
    JointKeyboardControl() : Node("joint_keyboard_control") {
        // 创建动作客户端，连接 arm_controller
        client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory");

        // 你的机械臂实际关节名（关键！适配 joint1-joint6）
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        // 初始关节角度（全部为0）
        joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // 每次按键的角度步长（0.1弧度，运动缓慢平稳）
        step_ = 0.1;

        // 打印操作提示（清晰易懂）
        print_help();
    }

    // 打印键盘操作说明
    void print_help() {
        RCLCPP_INFO(this->get_logger(), "=====================================");
        RCLCPP_INFO(this->get_logger(), "虚拟机械臂键盘控制（适配 joint1-joint6）");
        RCLCPP_INFO(this->get_logger(), "=====================================");
        RCLCPP_INFO(this->get_logger(), "按键功能：");
        RCLCPP_INFO(this->get_logger(), "1: joint1 +0.1 | 2: joint1 -0.1");
        RCLCPP_INFO(this->get_logger(), "3: joint2 +0.1 | 4: joint2 -0.1");
        RCLCPP_INFO(this->get_logger(), "5: joint3 +0.1 | 6: joint3 -0.1");
        RCLCPP_INFO(this->get_logger(), "7: joint4 +0.1 | 8: joint4 -0.1");
        RCLCPP_INFO(this->get_logger(), "9: joint5 +0.1 | 0: joint5 -0.1");
        RCLCPP_INFO(this->get_logger(), "-: joint6 +0.1 | =: joint6 -0.1");
        RCLCPP_INFO(this->get_logger(), "x: 重置所有关节到0 | ESC: 退出");
        RCLCPP_INFO(this->get_logger(), "=====================================");
        RCLCPP_INFO(this->get_logger(), "提示：按键后机械臂会缓慢运动，关节数据节点会同步打印角度！");
    }

    // 发送关节轨迹指令到控制器
    void send_joint_goal() {
        // 等待动作服务器就绪
        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "控制器未就绪，跳过本次指令");
            return;
        }

        // 构造轨迹指令（适配控制器要求）
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = joint_names_;
        goal_msg.trajectory.points.resize(1);
        goal_msg.trajectory.points[0].positions = joint_positions_;
        goal_msg.trajectory.points[0].velocities = std::vector<double>(6, 0.0);
        goal_msg.trajectory.points[0].accelerations = std::vector<double>(6, 0.0);
        goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.5); // 运动更快
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);

        // 发送指令（异步，不阻塞键盘输入）
        client_->async_send_goal(goal_msg);

        // 打印当前关节角度（方便查看）
        std::string pos_str = "当前关节角度：";
        for (size_t i=0; i<joint_names_.size(); i++) {
            pos_str += joint_names_[i] + ":" + std::to_string(joint_positions_[i]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", pos_str.c_str());
    }

    // 非阻塞读取键盘按键（核心）
    char get_key() {
        char c = 0;
        struct termios oldt, newt;
        // 保存原有终端设置
        tcgetattr(STDIN_FILENO, &oldt);
        // 设置非阻塞、无回显模式
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        // 读取按键
        c = getchar();
        // 恢复终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return c;
    }

    // 键盘控制主逻辑
    void run() {
        char key;
        while (rclcpp::ok()) {
            key = get_key();
            // 根据按键更新关节角度
            switch (key) {
                case '1': joint_positions_[0] += step_; break; // joint1 +
                case '2': joint_positions_[0] -= step_; break; // joint1 -
                case '3': joint_positions_[1] += step_; break; // joint2 +
                case '4': joint_positions_[1] -= step_; break; // joint2 -
                case '5': joint_positions_[2] += step_; break; // joint3 +
                case '6': joint_positions_[2] -= step_; break; // joint3 -
                case '7': joint_positions_[3] += step_; break; // joint4 +
                case '8': joint_positions_[3] -= step_; break; // joint4 -
                case '9': joint_positions_[4] += step_; break; // joint5 +
                case '0': joint_positions_[4] -= step_; break; // joint5 -
                case '-': joint_positions_[5] += step_; break; // joint6 +
                case '=': joint_positions_[5] -= step_; break; // joint6 -
                case 'x': joint_positions_ = {0.0,0.0,0.0,0.0,0.0,0.0}; break; // 重置
                case 27: rclcpp::shutdown(); return; // ESC退出
                default: continue; // 其他按键忽略
            }
            // 发送更新后的关节角度
            send_joint_goal();
            // 小延迟，避免指令发送过快
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    double step_;
};

int main(int argc, char *argv[]) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    // 创建节点
    auto node = std::make_shared<JointKeyboardControl>();
    // 启动键盘控制
    node->run();
    // 关闭节点
    rclcpp::shutdown();
    return 0;
}
