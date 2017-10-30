cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ir_camera_plugin)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  message_generation
)

find_package(gazebo REQUIRED)

add_message_files(
  FILES
  IRCamera.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    gazebo_ros_ir_camera
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_plugins
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

SET(camera_plugin_sources
    src/gazebo_ros_ir_camera_sensor.cpp
    include/gazebo_ros_ir_camera_plugin/gazebo_ros_ir_camera_sensor.h
)

SET(emitter_plugin_sources
    src/gazebo_ros_ir_emitter_model.cpp
    include/gazebo_ros_ir_camera_plugin/gazebo_ros_ir_emitter_model.h
)

add_library(gazebo_ros_ir_camera ${camera_plugin_sources})
target_link_libraries(gazebo_ros_ir_camera ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_ir_emitter ${emitter_plugin_sources})
target_link_libraries(gazebo_ros_ir_emitter ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS
  gazebo_ros_ir_camera
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
