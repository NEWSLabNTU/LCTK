cmake_minimum_required(VERSION 3.10)
project(multi_wayside_node)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/roi_interactive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()