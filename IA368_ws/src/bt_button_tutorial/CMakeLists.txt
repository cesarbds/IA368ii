cmake_minimum_required(VERSION 3.16)
project(bt_button_tutorial)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(bt_button_interfaces REQUIRED)
add_executable(button_tree
  src/button_tree.cpp
  src/wait.cpp
  src/wait_for_button.cpp
  src/print.cpp
)
target_include_directories(button_tree PRIVATE include)
ament_target_dependencies(button_tree ament_index_cpp behaviortree_cpp rclcpp std_msgs behaviortree_ros2 bt_button_interfaces)

add_executable(random_button_tree
  src/random_button_tree.cpp
  src/wait.cpp
  src/wait_for_button.cpp
  src/print.cpp
  src/random_odd.cpp
)
target_include_directories(random_button_tree PRIVATE include)
ament_target_dependencies(random_button_tree ament_index_cpp behaviortree_cpp rclcpp std_msgs behaviortree_ros2 bt_button_interfaces)

install(PROGRAMS scripts/button_server.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS button_tree random_button_tree DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
ament_package()
