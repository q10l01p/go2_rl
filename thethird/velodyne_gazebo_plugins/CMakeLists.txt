cmake_minimum_required(VERSION 3.0.2)
project(velodyne_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros)

catkin_package(LIBRARIES
  gazebo_ros_velodyne_laser
  gazebo_ros_velodyne_gpu_laser
  INCLUDE_DIRS include
)

add_library(gazebo_ros_velodyne_laser SHARED IMPORTED)
set_target_properties(gazebo_ros_velodyne_laser PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libgazebo_ros_velodyne_laser.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(gazebo_ros_velodyne_gpu_laser SHARED IMPORTED)
set_target_properties(gazebo_ros_velodyne_gpu_laser PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libgazebo_ros_velodyne_gpu_laser.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set(PREBUILT_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/libgazebo_ros_velodyne_laser.so"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/libgazebo_ros_velodyne_gpu_laser.so"
)

foreach(lib_path IN LISTS PREBUILT_LIBS)
  get_filename_component(lib_name "${lib_path}" NAME)
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib")
  configure_file(
    "${lib_path}"
    "${CATKIN_DEVEL_PREFIX}/lib/${lib_name}"
    COPYONLY
  )
endforeach()

install(FILES
  lib/libgazebo_ros_velodyne_laser.so
  lib/libgazebo_ros_velodyne_gpu_laser.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
