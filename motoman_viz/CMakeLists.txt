cmake_minimum_required(VERSION 2.8.3)
project(motoman_viz)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_move_group
  moveit_msgs
  motoman_viz_msgs
  tf
  tf_conversions
  eigen_conversions
  )

catkin_package(
  CATKIN_DEPENDS
  )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(-std=c++11)

# Build
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_executable(forward_kinematics_solver src/forward_kinematics_solver.cpp)

add_dependencies(forward_kinematics_solver forward_kinematics_generate_messages_cpp)
add_dependencies(forward_kinematics_solver ${catkin_EXPORTED_TARGETS})

target_link_libraries(forward_kinematics_solver
  ${catkin_LIBRARIES}
  )

# Install
foreach(modules forward_kinematics_solver)
  install(TARGETS ${modules}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach(modules)
