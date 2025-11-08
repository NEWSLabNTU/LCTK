cmake_minimum_required(VERSION 3.10)
project(lctk_launch)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(FILES
  # Modular component launch files
  launch/image_processing.launch.xml
  launch/pointcloud_processing.launch.xml
  launch/extrinsic_calibration.launch.xml
  launch/calibration_judge.launch.xml
  # Refactored calibration launch files
  launch/lidar_camera_calibration.launch.xml
  launch/two_lidar_calibration.launch.xml
  # Visualization launch files
  launch/rviz.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
# Use file(GLOB_RECURSE) to find all config files, which allows --symlink-install to work
file(GLOB_RECURSE CONFIG_FILES
  "config/*.json5"
  "config/*.yaml"
  "config/*.yml"
  "config/*.rviz"
  "config/README.md"
)

# Install each file preserving directory structure
foreach(CONFIG_FILE ${CONFIG_FILES})
  file(RELATIVE_PATH REL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config ${CONFIG_FILE})
  get_filename_component(REL_DIR ${REL_PATH} DIRECTORY)
  install(FILES ${CONFIG_FILE}
    DESTINATION share/${PROJECT_NAME}/config/${REL_DIR}
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
