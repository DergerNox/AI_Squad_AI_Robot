cmake_minimum_required(VERSION 3.10)
project(picar_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME})

ament_package()
