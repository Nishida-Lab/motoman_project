cmake_minimum_required(VERSION 2.8.3)
project(motoman_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  motoman_description
  roscpp
  roslaunch
  sensor_msgs
  trajectory_msgs
  )

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  )

# Build
add_definitions("-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motoman_control_node
  src/motoman_control_node.cpp
)

add_dependencies(motoman_control_node motoman_control_generate_messages_cpp)

target_link_libraries(motoman_control_node
  ${catkin_LIBRARIES}
)

# Install
foreach(modules motoman_control_node)
  install(TARGETS ${modules}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach(modules)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
