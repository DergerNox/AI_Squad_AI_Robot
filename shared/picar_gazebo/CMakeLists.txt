cmake_minimum_required(VERSION 3.10)
project(picar_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME})

# The twin's distance sensor node (node name: ultrasonic)
install(PROGRAMS scripts/sim_ultrasonic.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_ultrasonic test/test_sim_ultrasonic.py)
endif()

ament_package()
