cmake_minimum_required(VERSION 3.8)
project(cmd_vel_guard)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cmd_vel_filter src/cmd_vel_filter.cpp)
ament_target_dependencies(cmd_vel_filter rclcpp geometry_msgs std_msgs)

add_executable(robot_state_monitor src/robot_state_monitor.cpp)
ament_target_dependencies(robot_state_monitor rclcpp geometry_msgs std_msgs)



install(TARGETS cmd_vel_filter robot_state_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
ament_package()
