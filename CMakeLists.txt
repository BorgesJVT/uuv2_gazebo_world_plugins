cmake_minimum_required(VERSION 3.5)
project(uuv2_gazebo_world_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(ignition-math6 REQUIRED)

# BUILD
add_library(underwater_current_plugin SHARED
  src/UnderwaterCurrentPlugin.cpp
  src/GaussMarkovProcess.cpp
)

target_include_directories(underwater_current_plugin PUBLIC include)
ament_target_dependencies(underwater_current_plugin PUBLIC GAZEBO ignition-common3)

# EXPORT
ament_export_include_directories(include)
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

# TEST
if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(gazebo_ros REQUIRED)

  # ament_lint_auto_find_test_dependencies()
  add_launch_test(test/underwater_empty_world_spawn.test.py)

endif()

ament_package()
