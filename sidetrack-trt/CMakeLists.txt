cmake_minimum_required(VERSION 3.0.2)

project(side_track)

find_package(
    catkin REQUIRED
    roscpp
    cv_bridge
    genmsg
    sensor_msgs
    image_transport
)

add_message_files(
  FILES
  front_msgs.msg
)
generate_messages(DEPENDENCIES sensor_msgs)
catkin_package()

add_definitions(-std=c++11)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(CUDA REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_detect src/front_node.cpp src/side_lane_detect.cpp)
target_link_libraries(${PROJECT_NAME}_detect nvinfer cudart ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_detect ${PROJECT_NAME}_generate_messages_cpp)
# add_definitions(-O2 -pthread)

