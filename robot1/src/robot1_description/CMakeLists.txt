cmake_minimum_required(VERSION 3.8)
project(robot1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# 安装核心目录（URDF/SRDF/launch等，保留原有逻辑）
install(
  DIRECTORY launch rviz urdf meshes config srdf  # 新增srdf目录（如果有）
  DESTINATION share/${PROJECT_NAME}
)

# ========== 关键修改：修复Python脚本安装逻辑 ==========
# 方案1：如果暂时没有keyboard_absolute_pose.py，注释这部分（优先选这个）
# install(PROGRAMS
#   scripts/robot1_absolute_control.py  # 后续脚本放在scripts目录，路径更规范
#   DESTINATION lib/${PROJECT_NAME}
# )

# 方案2：如果要保留安装逻辑（后续添加脚本时启用）
# 先创建空的scripts目录，避免编译报错
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
# 安装scripts目录下的所有Python脚本（后续添加脚本后自动生效）
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

ament_package()
