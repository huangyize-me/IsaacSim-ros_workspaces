cmake_minimum_required(VERSION 3.5)
project(lumi_navigation_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装 Python 脚本（使其可执行）
install(PROGRAMS
  scripts/lumi_twist_to_joints.py
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件目录
install(DIRECTORY
  launch
  params
  maps
  rviz2
  DESTINATION share/${PROJECT_NAME})

ament_package()
