set (sources
  Geometry.cc
  Perceptors.cc
  states/BeforeKickOffState.cc
  states/CornerKickState.cc
  states/FreeKickState.cc
  states/GameOverState.cc
  states/GoalKickState.cc
  states/GoalState.cc
  states/KickInState.cc
  states/KickOffState.cc
  states/PlayOnState.cc
  states/State.cc
  GameState.cc
  Robocup3dsPlugin.cc
)

set (gtest_sources
  Robocup3dsPlugin_TEST.cc
  GameState_TEST.cc
  Perceptors_TEST.cc
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

target_link_libraries(${PROJECT_NAME_LOWER}Plugin
  gazebo_common
)

robocup3ds_build_tests(${gtest_sources})
