#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>  // 用于设置小数精度
// 新增：引入serial库头文件
#include <serial/serial.h>

// 定义节点日志器
static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_serial_publisher_node");

// 全局串口对象（也可以封装到类里，新手用全局更简单）
serial::Serial ser;

/**
 * @brief 初始化串口
 * @param port 串口设备名（如/dev/ttyACM0）
 * @param baudrate 波特率（如115200）
 * @return true-成功，false-失败
 */
bool initSerial(const std::string& port, int baudrate) {
    try {
        // 配置串口参数
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        // 设置超时：读超时1000ms，写超时1000ms
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        
        // 打开串口
        ser.open();
        
        if (ser.isOpen()) {
            RCLCPP_INFO(LOGGER, "串口 %s 打开成功！波特率：%d", port.c_str(), baudrate);
            return true;
        } else {
            RCLCPP_ERROR(LOGGER, "串口 %s 打开失败：无法确认状态", port.c_str());
            return false;
        }
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(LOGGER, "串口 %s 打开失败：%s", port.c_str(), e.what());
        return false;
    } catch (std::exception& e) {
        RCLCPP_ERROR(LOGGER, "串口初始化异常：%s", e.what());
        return false;
    }
}

/**
 * @brief 关节状态回调函数：接收/joint_states，通过串口发送
 */
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 空消息校验
    if (!msg || msg->name.empty() || msg->position.empty()) {
        RCLCPP_WARN(LOGGER, "收到空的关节状态消息，跳过处理！");
        return;
    }

    // 拼接关节数据（格式：关节名:角度,关节名:角度...）
    std::ostringstream joint_data_stream;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joint_data_stream << msg->name[i] << ":" << std::fixed << std::setprecision(4) << msg->position[i];
        if (i != msg->name.size() - 1) {
            joint_data_stream << ",";
        }
    }
    std::string joint_data = joint_data_stream.str();

    // 真实串口发送（核心）
    if (ser.isOpen()) {
        try {
            // 发送数据+换行符（方便下位机解析）
            ser.write(joint_data + "\n");
            RCLCPP_INFO(LOGGER, "【物理串口发送】关节角度：%s", joint_data.c_str());
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(LOGGER, "串口发送失败：%s", e.what());
        }
    } else {
        // 串口未打开时，降级为模拟打印（兼容虚拟模式）
        RCLCPP_WARN(LOGGER, "串口未打开，仅模拟发送：%s", joint_data.c_str());
    }
}

int main(int argc, char * argv[])
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joint_serial_publisher_node");

    // 配置串口参数（可根据实际修改）
    std::string serial_port = "/dev/ttyACM0";  // 替换为你的实际串口名
    int baudrate = 115200;                     // 与下位机一致

    // 打印启动提示
    RCLCPP_INFO(LOGGER, "=====================================");
    RCLCPP_INFO(LOGGER, "机械臂串口发送节点已启动！");
    RCLCPP_INFO(LOGGER, "待打开串口：%s，波特率：%d", serial_port.c_str(), baudrate);
    RCLCPP_INFO(LOGGER, "订阅话题：/joint_states");
    RCLCPP_INFO(LOGGER, "=====================================");

    // 初始化串口
    initSerial(serial_port, baudrate);

    // 订阅/joint_states话题
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 
        10, 
        std::bind(&jointStateCallback, std::placeholders::_1)
    );

    // 自旋等待回调
    rclcpp::spin(node);

    // 关闭节点：优雅关闭串口
    if (ser.isOpen()) {
        ser.close();
        RCLCPP_INFO(LOGGER, "串口已关闭！");
    }
    rclcpp::shutdown();
    return 0;
}
