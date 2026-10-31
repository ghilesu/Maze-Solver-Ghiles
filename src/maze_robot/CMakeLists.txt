cmake_minimum_required(VERSION 3.8)
project(maze_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_FLAGS MATCHES "-G")
  cmake_path(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    worlds
    meshes
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/maze_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
