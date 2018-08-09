cmake_minimum_required(VERSION 2.8.3)
project(motoman_point_cloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  cmake_modules
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  cmake_modules
  )

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros pcl_msgs tf tf_conversions
  )

# Build
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_COMMON_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(
  kinect_downsampler_node
  src/kinect_downsampler.cpp
  src/kinect_downsampler_node.cpp
  )

target_link_libraries(kinect_downsampler_node
  ${catkin_LIBRARIES}
)

add_executable(point_cloud_merger src/point_cloud_merger.cpp)
target_link_libraries(point_cloud_merger ${catkin_LIBRARIES})

# Install
foreach(modules kinect_downsampler_node point_cloud_merger)
  install(TARGETS ${modules}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach(modules)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
