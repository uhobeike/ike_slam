cmake_minimum_required(VERSION 3.8)
project(ike_slam)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map/sparseOccupancyGridMap.cpp
  src/mapping/mapping.cpp
  src/pf/likelihoodField.cpp
  src/pf/mcl.cpp
  src/pf/motionModel.cpp
  src/pf/observationModel.cpp
  src/pf/resampling.cpp
  src/scan_matching/scan_matching.cpp

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
