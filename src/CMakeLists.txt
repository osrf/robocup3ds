
set(PARSER_SOURCE ../lib/sexpLibrary/parser.c ../lib/sexpLibrary/sexp.c ../lib/sexpLibrary/event_temp.c ../lib/sexpLibrary/sexp_ops.c ../lib/sexpLibrary/cstring.c ../lib/sexpLibrary/faststack.c)
Set(PARSER_HEADER ../lib/sexpLibrary/sexp.h ../lib/sexpLibrary/cstring.h ../lib/sexpLibrary/faststack.h ../lib/sexpLibrary/sexp_errors.h ../lib/sexpLibrary/sexp_ops.h )


set (sources
  Effector.cc
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
  Geometry.cc
  Perceptors.cc
  Robocup3dsPlugin.cc
  Server.cc
  Util.cc
)

set (gtest_sources
  Effector_TEST.cc
  Robocup3dsPlugin_TEST.cc
  Server_TEST.cc
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

robocup3ds_add_library(${PROJECT_NAME_LOWER}Plugin ${sources} ${PARSER_SOURCE} ${HEADER_SOURCE})
robocup3ds_install_library(${PROJECT_NAME_LOWER}Plugin)

set(GAZEBO_LIBRARIES_ROBOCUP3DS   libgazebo_client.so   libgazebo_gui_building.so   libgazebo_gui_viewers.so   libgazebo_gui_model.so   libgazebo_gui.so  libgazebo_sensors.so  libgazebo_rendering.so  libgazebo_selection_buffer.so  libgazebo_physics_dart.so  libgazebo_physics_bullet.so  libgazebo_physics_simbody.so  libgazebo_player.so  libgazebo_physics_ode.so  libgazebo_physics.so  libgazebo_ode.so  libgazebo_rendering_deferred.so  libgazebo_transport.so  libgazebo_msgs.so  libgazebo_util.so  libgazebo_common.so  libgazebo_skyx.so  libgazebo_gimpact.so  libgazebo_opcode.so  libgazebo_opende_ou.so  libgazebo_math.so optimized protobuf.so debug   protobuf.so -lpthread  sdformat.so  libignition-math2.so optimized   OgreMain.so debug   OgreMain.so optimized   OgreTerrain.so debug   OgreTerrain.so optimized   OgrePaging.so debug   OgrePaging.so  libignition-math2.so libgazebo_sensors.so)

target_link_libraries(${PROJECT_NAME_LOWER}Plugin
  ${GAZEBO_LIBRARIES_ROBOCUP3DS}
  ${IGNITION-MATH_LIBRARIES}
)


robocup3ds_build_tests(${gtest_sources})
