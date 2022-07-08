cmake_minimum_required(VERSION 3.0.2)
project(psdet_linebase)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs 
    genmsg
    geometry_msgs
    image_transport
    pluginlib
    cv_bridge
    sensor_msgs
    tf
)

generate_messages(
    DEPENDENCIES
    geometry_msgs 
    std_msgs
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#OpenCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)



# cuda
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
cuda_add_library(myplugins SHARED src/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

file(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}_node 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    nvinfer 
    cudart
    myplugins
)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

if(UNIX)
add_definitions(-O2 -pthread)
endif(UNIX)