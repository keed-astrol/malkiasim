cmake_minimum_required(VERSION 3.8)
project(dojo_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# --- Python nodes (pure Python, installed via setup.py) ---
ament_python_install_package(${PROJECT_NAME})

# --- C++ BT plugins (shared library loaded by Nav2 bt_navigator) ---
set(dependencies
  rclcpp
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp
)

add_library(dojo_bt_plugins SHARED
  src/bt_conditions.cpp
  src/bt_actions.cpp
)
target_include_directories(dojo_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dojo_bt_plugins ${dependencies})
target_include_directories(dojo_bt_plugins PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(dojo_bt_plugins ${JSONCPP_LIBRARIES})
target_compile_definitions(dojo_bt_plugins PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS dojo_bt_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Behavior tree XML files ---
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

# --- Config files ---
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# --- Launch files ---
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- Python package ---
install(DIRECTORY ${PROJECT_NAME} DESTINATION share/${PROJECT_NAME})

ament_package()
