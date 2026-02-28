cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 安装配置文件和启动文件
install(
  DIRECTORY 
    config  # 配置文件目录
    launch  # 启动文件目录
  DESTINATION share/${PROJECT_NAME}
)

# 如果有需要安装的其他目录，可以在这里添加
# install(
#   DIRECTORY 
#     worlds  # Gazebo 世界文件
#     maps    # 地图文件
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()