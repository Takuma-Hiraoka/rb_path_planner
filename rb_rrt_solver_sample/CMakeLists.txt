cmake_minimum_required(VERSION 3.0.2)
project(rb_rrt_solver_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  ik_constraint2_vclip
  ik_constraint2_distance_field
  rb_rrt_solver
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

add_library(CnoidRBRRTSolverSamplePlugin SHARED
  src/CnoidRBRRTSolverSamplePlugin.cpp
  src/sample1_4limb.cpp
  )
target_link_libraries(CnoidRBRRTSolverSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  )
set_target_properties(CnoidRBRRTSolverSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
