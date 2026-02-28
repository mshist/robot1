# 指定最低CMake版本（ROS2 Humble要求≥3.8，这里适配所有ROS2版本）
cmake_minimum_required(VERSION 3.8)
# 项目名（与package.xml一致，必须）
project(joint_serial_node)

# ========== 编译选项：开启严格警告，提前发现代码问题 ==========
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # 新增 -Wno-unused-parameter：避免"未使用参数"的无关警告（ROS2回调常用）
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ========== 查找依赖：按"核心→功能→消息→硬件"顺序 ==========
# 基础依赖（必须）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 功能依赖（键盘控制节点需要）
find_package(rclcpp_action REQUIRED)   # ROS2动作通信
find_package(control_msgs REQUIRED)    # 关节轨迹动作消息

# 消息依赖（关节数据采集节点需要）
find_package(sensor_msgs REQUIRED)     # JointState消息

# 硬件依赖（串口通信需要，新增）
find_package(serial REQUIRED)          # serial库：对接物理串口

# ========== 编译可执行文件 ==========
# 节点1：关节数据采集+串口发送（新增serial链接）
add_executable(joint_to_serial 
  src/joint_to_serial.cpp  # 明确路径，避免文件重名问题
)
# 绑定依赖：新增serial，让节点能调用串口API
ament_target_dependencies(joint_to_serial
  rclcpp
  sensor_msgs
  serial  # 新增：串口依赖
)
# 链接serial库（关键！否则会报"未定义引用"错误）
target_link_libraries(joint_to_serial ${serial_LIBRARIES})

# 节点2：键盘控制关节（动作客户端）
add_executable(joint_keyboard_control 
  src/joint_keyboard_control.cpp
)
ament_target_dependencies(joint_keyboard_control
  rclcpp
  rclcpp_action
  control_msgs
)

# ========== 安装可执行文件：ROS2标准路径 ==========
install(TARGETS
  joint_to_serial
  joint_keyboard_control
  # 安装到lib/包名目录下，ros2 run 能直接找到
  DESTINATION lib/${PROJECT_NAME}
)

# ========== 测试配置：保持默认即可 ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查（新手无需纠结），其他lint正常运行
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========== 生成ROS2包配置文件（必须） ==========
ament_package()
