set (sources
  Robocup3dsPlugin.cc
)

set (sources
  Server.cc
)

set (gtest_sources
  Robocup3dsPlugin_TEST.cc
)

set (gtest_sources
  Server_TEST.cc
)

# make below target specific?
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

robocup3ds_add_library(${PROJECT_NAME_LOWER}Plugin ${sources})
robocup3ds_install_library(${PROJECT_NAME_LOWER}Plugin)

robocup3ds_build_tests(${gtest_sources})
