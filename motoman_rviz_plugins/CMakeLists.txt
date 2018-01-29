cmake_minimum_required(VERSION 2.8.3)
project(motoman_rviz_plugins)
find_package(catkin REQUIRED
  COMPONENTS
  rviz
  motoman_viz_msgs
  message_generation
  jsk_topic_tools
  )

set(CMAKE_CXX_FLAGS "-std=c++11")

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

catkin_package(
    DEPENDS rviz
    CATKIN_DEPENDS motoman_viz_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS -g -DQT_QMAKE_EXECUTABLE=/usr/bin/qmake-qt4)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

qt4_wrap_cpp(MOC_FILES
  src/euclidean_link_trajectory.h
  src/bounding_box_display_common.h
  src/bounding_box_array_display.h
  src/bounding_box_display.h
  )

set(SOURCE_FILES
  src/euclidean_link_trajectory.cpp
  src/bounding_box_array_display.cpp
  src/bounding_box_display.cpp
  ${MOC_FILES}
  )

add_library(motoman_rviz_plugins ${SOURCE_FILES} ${UIC_FILES})
target_link_libraries(motoman_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
add_dependencies(motoman_rviz_plugins motoman_viz_msgs_generate_messages_cpp ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(motoman_rviz_plugins PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS motoman_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(DIRECTORY ./src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h")
