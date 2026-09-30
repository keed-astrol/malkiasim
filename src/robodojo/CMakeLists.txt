cmake_minimum_required(VERSION 3.8)
project(robodojo)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(
  PROGRAMS
    robodojo/joint_state_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_publisher_custom
)

install(
  DIRECTORY config launch meshes textures urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
