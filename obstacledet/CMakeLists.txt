cmake_minimum_required(VERSION 3.0.2)

project(obstacle_det)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  sensor_msgs
  message_generation
  pcl_conversions 
  pcl_ros
  rviz
)

find_package( OpenCV REQUIRED )

add_message_files(FILES 
  CircleObstacle.msg 
  SegmentObstacle.msg 
  Obstacles.msg
)
generate_messages(
  DEPENDENCIES 
  sensor_msgs
  geometry_msgs
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_gui
  CATKIN_DEPENDS roscpp message_runtime rviz
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


#
# Build rviz plugins
#
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}_gui
  src/displays/obstacles_display.cpp include/displays/obstacles_display.h
  src/displays/circle_visual.cpp include/displays/circle_visual.h
  src/displays/segment_visual.cpp include/displays/segment_visual.h
  )
target_link_libraries(${PROJECT_NAME}_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})




add_executable(${PROJECT_NAME}_rgbd_proc 
src/rgbd_proc.cpp
src/ground_seg.cpp
)
target_link_libraries(${PROJECT_NAME}_rgbd_proc 
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_rgbd_proc 
  ${PROJECT_NAME}_generate_messages_cpp
)


add_executable(${PROJECT_NAME}_reg 
src/obstacle_reg.cpp
# src/ground_seg.cpp
)
target_link_libraries(${PROJECT_NAME}_reg 
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_reg 
  ${PROJECT_NAME}_generate_messages_cpp
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_rgbd_proc ${PROJECT_NAME}_reg ${PROJECT_NAME}_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Cpack ##
#############

string(TIMESTAMP COMPILE_TIME %Y%m%d-%H%M%S)
set(build_time    ${COMPILE_TIME})

set(CPACK_GENERATOR "DEB")

#Creating UNIX structured package
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/")
set(CPACK_PACKAGE_VERSION 0.0.1-${build_time})

#Set directory to /../ to avoid /share part of ROS_PACKAGE_PATH
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};ALL;/./")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "${PROJECT_NAME} - ${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "RCS")

#Now we can simply create the dependencies using almost the same package names as they declared in ROS repository
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-kinetic-roscpp, ros-kinetic-std-msgs, ros-kinetic-message-runtime, ros-kinetic-sensor-msgs, ros-kinetic-libmodbus")

#IMPORTANT: set this to keep on auto-introspection for system dependencies like libc++6
#set (CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

#Keep Debian package structure untouched
set(CPACK_DEBIAN_PACKAGE_CONTROL_STRICT_PERMISSION TRUE)

#Standard naming schema for Debian-based distros
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}~${CMAKE_SYSTEM_NAME}_${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_VENDOR "Technical University")
set(CPACK_PACKAGE_CONTACT "<your@email.com>")

#Turn the process on    
include(CPack)

