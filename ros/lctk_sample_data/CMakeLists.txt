cmake_minimum_required(VERSION 3.10)
project(lctk_sample_data)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch.xml"
)

# No data/ install rule: the recordings moved into the sessions that describe
# them (sessions/<name>/data), so each calibration session is self-contained on
# disk. This package now ships launch files only.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()