cmake_minimum_required(VERSION 3.22)
project(moveit2_ur5e)

find_package(ament_cmake REQUIRED)
add_executable(moveit2_ur5e src/hello_moveit.cpp)
target_include_directories(moveit2_ur5e PUBLIC include)

ament_target_dependencies(
  moveit2_ur5e
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
