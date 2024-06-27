cmake_minimum_required(VERSION 3.0.2)
project(aik_constraint)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)

find_package(choreonoid REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aik_constraint
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS CHOREONOID
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Jacobian.cpp
  src/Force.cpp
  src/IKConstraint.cpp
  src/PositionConstraint.cpp
  src/RegionConstraint.cpp
  src/COMConstraint.cpp
  # src/COMVelocityConstraint.cpp
  src/AngularMomentumConstraint.cpp
  # src/AngularMomentumLimitConstraint.cpp
  src/JointAngleConstraint.cpp
  src/JointLimitConstraint.cpp
  # src/JointVelocityConstraint.cpp
  src/CollisionConstraint.cpp
  src/ClientCollisionConstraint.cpp
  src/ForceConstraint.cpp
  src/EOMConstraint.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ik_constraint.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
