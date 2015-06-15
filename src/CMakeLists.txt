set (sources
  Geometry.cc
  states/BeforeKickOffState.cc
  states/CornerKickLeftState.cc
  states/CornerKickRightState.cc
  states/FreeKickLeftState.cc
  states/FreeKickRightState.cc
  states/GameOverState.cc
  states/GoalKickLeftState.cc
  states/GoalKickRightState.cc
  states/GoalLeftState.cc
  states/GoalRightState.cc
  states/KickInLeftState.cc
  states/KickInRightState.cc
  states/KickOffLeftState.cc
  states/KickOffRightState.cc
  states/PlayState.cc
  states/State.cc
  GameState.cc
  Robocup3dsPlugin.cc
)

set (gtest_sources
  Robocup3dsPlugin_TEST.cc
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
