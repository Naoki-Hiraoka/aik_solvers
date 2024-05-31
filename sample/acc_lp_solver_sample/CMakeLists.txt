cmake_minimum_required(VERSION 3.0.2)
project(acc_lp_solver_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  aik_constraint
  acc_lp_solver
  roslib
  choreonoid_viewer
  )
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidAccLpSolverSamplePlugin SHARED
  src/CnoidAccLpSolverSamplePlugin.cpp
  src/sample1.cpp
)
target_link_libraries(CnoidAccLpSolverSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidAccLpSolverSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
