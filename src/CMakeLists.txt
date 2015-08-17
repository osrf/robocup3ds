set(PARSER_SOURCE ../lib/sexpLibrary/parser.c ../lib/sexpLibrary/sexp.c ../lib/sexpLibrary/event_temp.c ../lib/sexpLibrary/sexp_ops.c ../lib/sexpLibrary/cstring.c ../lib/sexpLibrary/faststack.c)
Set(PARSER_HEADER ../lib/sexpLibrary/sexp.h ../lib/sexpLibrary/cstring.h ../lib/sexpLibrary/faststack.h ../lib/sexpLibrary/sexp_errors.h ../lib/sexpLibrary/sexp_ops.h ../lib/sexp_memory.h)

set (sources
  Geometry.cc
  Effector.cc
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
  Server.cc
  Util.cc
  ../test/integration/ClientAgent.cc
)

set (gtest_sources
  Agent_TEST.cc
  Geometry_TEST.cc
  Robocup3dsPlugin_TEST.cc
  Server_TEST.cc
  Util_TEST.cc
  Perceptors_TEST.cc
  Geometry_TEST.cc
  GameState_TEST.cc
)

# make below target specific?
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

robocup3ds_add_library(${PROJECT_NAME_LOWER}Plugin ${sources} ${PARSER_SOURCE})

target_link_libraries(${PROJECT_NAME_LOWER}Plugin
  gazebo_common
  gazebo_physics
  gazebo_sensors
  ${GAZEBO_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
)

robocup3ds_install_library(${PROJECT_NAME_LOWER}Plugin)

robocup3ds_build_tests(${gtest_sources})
