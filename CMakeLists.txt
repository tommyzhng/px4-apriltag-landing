cmake_minimum_required(VERSION 3.0.2)
project(px4_apriltag_landing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  apriltag_ros
  # Add other dependencies if needed
)

find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/px4_apriltag_landing
  include/px4_apriltag_landing
  ${catkin_INCLUDE_DIRS}
)

add_library(px4_apriltag_landing src/px4_apriltag_landing.cpp)
target_include_directories(px4_apriltag_landing PUBLIC include)
target_compile_features(px4_apriltag_landing PUBLIC cxx_std_17)

add_executable(px4_apriltag_landing_node src/px4_apriltag_landing_node.cpp)
target_link_libraries(px4_apriltag_landing_node PUBLIC ${catkin_LIBRARIES} px4_apriltag_landing)
add_dependencies(px4_apriltag_landing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
