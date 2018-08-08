cmake_minimum_required(VERSION 2.8.12)
project(choreonoid_ros_samples)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  choreonoid
  #choreonoid_rosplugin
  )

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${CHOREONOID_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

function(add_cnoid_simple_controller)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES})
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller PREFIX "")
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)
endfunction()

add_subdirectory(src)
