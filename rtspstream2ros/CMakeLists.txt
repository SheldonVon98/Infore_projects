cmake_minimum_required(VERSION 3.0.2)

project(rtsp_stream)

find_package(
  catkin REQUIRED
  roscpp
  genmsg
  sensor_msgs
  image_transport
  camera_info_manager
  cv_bridge
)

generate_messages(DEPENDENCIES sensor_msgs)
catkin_package()


###################### FFMPEG ########################
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVUTIL_INCLUDE_DIR libavutil/imgutils.h)
find_library(AVUTIL_LIBRARY imgutils)

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)
#######################################################

find_package(OpenCV)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
  ${AVCODEC_INCLUDE_DIR} 
  ${AVFORMAT_INCLUDE_DIR} 
  ${AVUTIL_INCLUDE_DIR}
  ${SWSCALE_INCLUDE_DIR}
)

add_executable(
  ${PROJECT_NAME}_node 
  src/stream_node.cpp 
  src/retrieve.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES}
  ${AVCODEC_LIBRARY} 
  ${AVFORMAT_LIBRARY} 
  ${AVUTIL_LIBRARY} 
  ${SWSCALE_LIBRARY} 
)
add_dependencies(
  ${PROJECT_NAME}_node 
  ${PROJECT_NAME}_generate_messages_cpp
)

