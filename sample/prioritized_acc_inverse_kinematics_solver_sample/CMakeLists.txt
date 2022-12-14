cmake_minimum_required(VERSION 3.0.2)
project(prioritized_acc_inverse_kinematics_solver_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  aik_constraint
  aik_constraint_vclip
  prioritized_acc_inverse_kinematics_solver
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

# 相対パスを絶対パスに直す
set(CHOREONOID_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_LIBRARIES_ABS ${CHOREONOID_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

add_executable(sampleSR1
  src/SampleSR1.cpp
)

target_link_libraries(sampleSR1
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  )

install(TARGETS sampleSR1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


add_library(CnoidPrioritizedAccInverseKinematicsSolverSamplePlugin SHARED
  src/CnoidPrioritizedAccInverseKinematicsSolverSamplePlugin.cpp
  src/sample1_4limb.cpp
  src/sample2_4limb_unsolvable.cpp
  src/sample3_4limb_angle.cpp
  src/sample4_4limb_move.cpp
  src/sample5_com.cpp
  src/sample6_wolimit.cpp
  src/sample7_limit.cpp
  src/sample8_collision.cpp
  src/sample9_collision_move.cpp
  src/sample10_angular.cpp
)
target_link_libraries(CnoidPrioritizedAccInverseKinematicsSolverSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  )
set_target_properties(CnoidPrioritizedAccInverseKinematicsSolverSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
