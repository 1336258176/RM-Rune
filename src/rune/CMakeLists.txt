cmake_minimum_required(VERSION 3.8)
project(rune)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Ceres REQUIRED)
ament_auto_find_build_dependencies()

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

ament_auto_add_library(recognition_node SHARED
  DIRECTORY src/recognition)
target_include_directories(recognition_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(recognition_node
  ${OpenCV_LIBS})
rclcpp_components_register_node(recognition_node
  PLUGIN rune::RecognitionNode
  EXECUTABLE recognition_module
)

ament_auto_add_library(transform_node SHARED
  DIRECTORY src/transform)
target_include_directories(transform_node PUBLIC ${OpenCV_INCLUDE_DIRS})
rclcpp_components_register_node(transform_node
  PLUGIN rune::TransformNode
  EXECUTABLE transform_module
)

ament_auto_add_library(prediction_node SHARED
  DIRECTORY src/prediction)
target_include_directories(prediction_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(prediction_node
  Ceres::ceres)
rclcpp_components_register_node(prediction_node
  PLUGIN rune::PredictionNode
  EXECUTABLE prediction_module
)

ament_auto_add_library(tracker_node SHARED
  DIRECTORY src/tracker)
target_include_directories(tracker_node PUBLIC ${OpenCV_INCLUDE_DIRS})
rclcpp_components_register_node(tracker_node
  PLUGIN rune::TrackerNode
  EXECUTABLE tracker_module
)

ament_auto_add_library(test_node SHARED
  DIRECTORY src/test)
target_include_directories(test_node PUBLIC ${OpenCV_INCLUDE_DIRS})
rclcpp_components_register_node(test_node
    PLUGIN rune::TestNode
    EXECUTABLE test_module
)

# install(DIRECTORY video DESTINATION share/${PROJECT_NAME}) # 测试用视频

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)