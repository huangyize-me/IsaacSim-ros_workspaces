cmake_minimum_required(VERSION 3.8)
project(robotiq_2f_85_gripper_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
