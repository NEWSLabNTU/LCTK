cmake_minimum_required(VERSION 3.10)
project(calib_launch)

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
  # Sensor launch files
  launch/lidar.launch.xml
  launch/camera.launch.xml
  launch/sensor.launch.xml
  # Modular component launch files
  launch/image_processing.launch.xml
  launch/pointcloud_processing.launch.xml
  launch/extrinsic_calibration.launch.xml
  launch/calibration_visualization.launch.xml
  # Refactored calibration launch files
  launch/lidar_camera_calibration.launch.xml
  launch/two_lidar_calibration.launch.xml
  # Visualization launch files
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
  PATTERN "*.json5"
  PATTERN "*.yaml"
  PATTERN "*.yml"
  PATTERN "README.md"
)

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
