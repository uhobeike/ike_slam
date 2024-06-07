cmake_minimum_required(VERSION 3.8)
project(ike_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map/sparseOccupancyGridMap.cpp
  src/pf/likelihoodField.cpp
  src/pf/mapping.cpp
  src/pf/mcl.cpp
  src/pf/motionModel.cpp
  src/pf/observationModel.cpp
  src/pf/resampling.cpp
  src/ike_slam.cpp
)

target_link_libraries(${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "ike_slam::IkeSlam"
  EXECUTABLE ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
