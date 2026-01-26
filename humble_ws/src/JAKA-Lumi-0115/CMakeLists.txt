cmake_minimum_required(VERSION 3.22)
project(JAKA-Lumi-0115)

find_package(ament_cmake REQUIRED)

# Install meshes
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install urdf
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
