cmake_minimum_required(VERSION 3.5)
project(uuv2_gazebo_world_plugins)

find_package(ament_cmake REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(ignition-common3 REQUIRED)

# BUILD
add_library(underwater_current_plugin SHARED
  src/UnderwaterCurrentPlugin.cpp
  src/GaussMarkovProcess.cpp
)

target_include_directories(underwater_current_plugin PUBLIC include)
ament_target_dependencies(underwater_current_plugin PUBLIC GAZEBO ignition-common3)

# EXPORT
ament_export_libraries(underwater_current_plugin)
ament_export_dependencies(GAZEBO ignition-common3)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  underwater_current_plugin 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
