cmake_minimum_required(VERSION 3.0.2)
project(multicontact_locomotion_planner_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  multicontact_locomotion_planner
  roslib
  choreonoid_viewer
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

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

add_library(CnoidMultiContactLocomotionPlannerSamplePlugin SHARED
  src/CnoidMultiContactLocomotionPlannerSamplePlugin.cpp
  src/samplerobot_common.cpp
  src/world_common.cpp
  src/sample1_walk.cpp
  src/sample2_quad.cpp
  )
target_link_libraries(CnoidMultiContactLocomotionPlannerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  )
set_target_properties(CnoidMultiContactLocomotionPlannerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
