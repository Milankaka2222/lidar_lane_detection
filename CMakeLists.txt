cmake_minimum_required(VERSION 3.0.2)
project(lidar_lane_detection)

# Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Link libraries
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

# Add executable
add_executable(lidar_lane_detection src/lidar_lane_detection.cpp)

# Link libraries to executable
target_link_libraries(lidar_lane_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Add dependencies
add_dependencies(lidar_lane_detection
  ${catkin_EXPORTED_TARGETS}
  ${PCL_EXPORTED_TARGETS}
)



