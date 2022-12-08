find_package(catkin REQUIRED COMPONENTS)
find_package(choreonoid REQUIRED)

# 相対パスを絶対パスに直す
set(CHOREONOID_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_LIBRARIES_ABS ${CHOREONOID_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(choreonoid_viewer_sample_lib
  choreonoid_viewer_sample_lib.cpp
)
target_link_libraries(choreonoid_viewer_sample_lib
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  choreonoid_viewer
  )

add_executable(choreonoid_viewer_sample
  choreonoid_viewer_sample.cpp
  )
target_link_libraries(choreonoid_viewer_sample
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  choreonoid_viewer_sample_lib
  )

add_library(CnoidViewerSamplePlugin SHARED
  CnoidViewerSamplePlugin.cpp
  )
target_link_libraries(CnoidViewerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  choreonoid_viewer_sample_lib
  )
set_target_properties(CnoidViewerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
