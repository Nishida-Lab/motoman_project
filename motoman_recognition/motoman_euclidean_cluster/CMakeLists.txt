cmake_minimum_required(VERSION 2.8.3)
project(motoman_euclidean_cluster)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  motoman_viz_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS 
    motoman_viz_msgs 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
    std_msgs 
    roscpp 
    rospy 
    tf
  DEPENDS OpenCV PCL 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(
  euclidean_cluster_node
  src/euclidean_cluster.cpp
  src/euclidean_cluster_node.cpp
  )

add_dependencies(euclidean_cluster_node motoman_viz_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(euclidean_cluster_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install
foreach(modules euclidean_cluster_node)
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
