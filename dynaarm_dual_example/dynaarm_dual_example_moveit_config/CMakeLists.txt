cmake_minimum_required(VERSION 3.22)
project(dynaarm_dual_example_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()


install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
