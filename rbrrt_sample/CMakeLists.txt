cmake_minimum_required(VERSION 3.0.2)
project(rbrrt_sample)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rbrrt
  choreonoid_viewer
  reachability_map_visualizer
  choreonoid_bullet
  choreonoid_cddlib
  roslib
  )

find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidRBRRTSamplePlugin SHARED
  src/CnoidRBRRTSamplePlugin.cpp
  src/view.cpp
  src/rmap.cpp
  src/walk.cpp
  src/jaxon_common.cpp
  )
target_link_libraries(CnoidRBRRTSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidRBRRTSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
