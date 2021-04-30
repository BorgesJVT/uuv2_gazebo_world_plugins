cmake_minimum_required(VERSION 3.5)
project(uuv2_gazebo_world_plugins)

find_package(gazebo REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(uuv_underwater_current_plugin SHARED
  src/UnderwaterCurrentPlugin.cc
  src/GaussMarkovProcess.cc
)

target_link_libraries(uuv_underwater_current_plugin ${ignition-common3_LIBRARIES})

ament_export_dependencies(uuv2_gazebo_world_plugins)
ament_package()

install(TARGETS 
  uuv_underwater_current_plugin 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
)