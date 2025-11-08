cmake_minimum_required(VERSION 3.10)
project(lctk_sample_data)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch.xml"
)

# Install sample data files
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()