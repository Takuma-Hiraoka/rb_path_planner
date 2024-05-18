cmake_minimum_required(VERSION 3.0.2)
project(multicontact_locomotion_planner_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  multicontact_locomotion_planner
  roslib
  choreonoid_viewer
  convex_polyhedron_intersection
  choreonoid_qhull
  ik_constraint2_bullet
  choreonoid_cddlib
  ik_constraint2
  )
find_package(choreonoid REQUIRED)
find_package(ompl REQUIRED)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
)

add_library(CnoidMultiContactLocomotionPlannerSamplePlugin SHARED
  src/CnoidMultiContactLocomotionPlannerSamplePlugin.cpp
  src/samplerobot_common.cpp
  src/world_common.cpp
  src/sample0_display.cpp
  src/sample1_walk.cpp
  src/sample2_quad.cpp
  src/sample3_desk.cpp
  src/sample4_ladder.cpp
  )
target_link_libraries(CnoidMultiContactLocomotionPlannerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidMultiContactLocomotionPlannerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
