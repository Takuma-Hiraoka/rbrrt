cmake_minimum_required(VERSION 3.0.2)
project(rbrrt)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  choreonoid_viewer
  ik_constraint2
  ik_constraint2_distance_field
  ik_constraint2_bullet
  prioritized_inverse_kinematics_solver2
  trajectory_optimizer
  reachability_map_visualizer
  choreonoid_qhull
  )

find_package(choreonoid REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rbrrt
  CATKIN_DEPENDS ik_constraint2 ik_constraint2_distance_field ik_constraint2_bullet prioritized_inverse_kinematics_solver2 trajectory_optimizer reachability_map_visualizer choreonoid_qhull
  )

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
 )

add_library(${PROJECT_NAME}
  src/rbrrt.cpp
  src/util.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
