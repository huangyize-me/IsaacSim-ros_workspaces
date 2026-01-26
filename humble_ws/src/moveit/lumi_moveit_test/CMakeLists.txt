cmake_minimum_required(VERSION 3.22)
project(lumi_moveit_test)

find_package(ament_cmake REQUIRED)

# Install launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install config files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install rviz files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
