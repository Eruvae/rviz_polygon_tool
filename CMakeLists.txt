cmake_minimum_required(VERSION 2.8.3)
project(rviz_polygon_tool)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs rviz
)

######
# QT #
######
set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#############
# LIBRARIES #
#############
# Header files as explained in https://answers.ros.org/question/215487/could-not-load-panel-in-rviz-pluginlibfactory-the-plugin-for-class/
set(HEADER_FILES
  include/rviz_polygon_tool/polygon_tool.h
)
set(SRC_FILES
  src/polygon_tool.cc
)
add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
