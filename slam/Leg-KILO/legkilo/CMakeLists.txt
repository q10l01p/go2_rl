cmake_minimum_required(VERSION 3.5)
project(legkilo)

set(CMAKE_BUILD_TYPE "Release") 


message(INFO "CMake build type:  ${CMAKE_BUILD_TYPE}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions  -ggdb ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

message(INFO "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(CPU_N)
message(INFO "Processer number:  ${CPU_N}")

# Boost checks PTHREAD_STACK_MIN in preprocessor conditionals but glibc 2.34+
# exposes it as a runtime sysconf() call, so provide a numeric fallback.
execute_process(
    COMMAND getconf PTHREAD_STACK_MIN
    OUTPUT_VARIABLE LEGKILO_PTHREAD_STACK_MIN
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE LEGKILO_PTHREAD_STACK_MIN_RESULT
)
if(LEGKILO_PTHREAD_STACK_MIN_RESULT EQUAL 0)
    string(REGEX MATCH "^[0-9]+$"
           LEGKILO_PTHREAD_STACK_MIN_IS_NUMBER
           "${LEGKILO_PTHREAD_STACK_MIN}")
    if(LEGKILO_PTHREAD_STACK_MIN_IS_NUMBER)
        set(LEGKILO_PTHREAD_STACK_MIN_DEFINE "${LEGKILO_PTHREAD_STACK_MIN}")
    endif()
endif()
if(NOT LEGKILO_PTHREAD_STACK_MIN_DEFINE)
    set(LEGKILO_PTHREAD_STACK_MIN_DEFINE 16384)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    unitree_legged_msgs
)

include_directories(
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS  roscpp  std_msgs sensor_msgs nav_msgs unitree_legged_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

set(SOURCES
    # Preprocessing modules
    src/preprocess/lidar_processing.cc
    src/preprocess/kinematics.cc
    
    # Core algorithms
    src/core/slam/KILO.cc
    src/core/slam/voxel_map.cc
    src/core/slam/eskf.cc
    
    # ROS interface
    src/interface/ros1/ros_interface.cc
    src/interface/ros1/options.cc
)

add_library(${PROJECT_NAME} ${SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE PTHREAD_STACK_MIN=${LEGKILO_PTHREAD_STACK_MIN_DEFINE})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    yaml-cpp
    glog
    gflags
)

add_executable(${PROJECT_NAME}_node
    src/apps/leg_kilo_node.cc
)
target_compile_definitions(${PROJECT_NAME}_node PRIVATE PTHREAD_STACK_MIN=${LEGKILO_PTHREAD_STACK_MIN_DEFINE})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}  
)
