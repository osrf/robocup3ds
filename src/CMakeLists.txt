
set(PARSER_SOURCE sexpLibrary/parser.c sexpLibrary/sexp.c sexpLibrary/event_temp.c sexpLibrary/sexp_ops.c sexpLibrary/cstring.c sexpLibrary/faststack.c)
Set(PARSER_HEADER sexpLibrary/sexp.h sexpLibrary/cstring.h sexpLibrary/faststack.h sexpLibrary/sexp_errors.h sexpLibrary/sexp_ops.h )


set (sources
  Geometry.cc
  Effectors.cc
  Perceptors.cc
  Server.cc
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
  Effectors_TEST.cc
  Robocup3dsPlugin_TEST.cc
  GameState_TEST.cc
  Perceptors_TEST.cc
  Geometry_TEST.cc
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

robocup3ds_add_library(${PROJECT_NAME_LOWER}Plugin ${sources} ${PARSER_SOURCE} ${HEADER_SOURCE})
robocup3ds_install_library(${PROJECT_NAME_LOWER}Plugin)

target_link_libraries(${PROJECT_NAME_LOWER}Plugin
  gazebo_sensors
  gazebo_common
  ${IGNITION-MATH_LIBRARIES}
)

robocup3ds_build_tests(${gtest_sources})
