set (sources
  Effector.cc
  GameState.cc
  Geometry.cc
  Perceptors.cc
  Robocup3dsPlugin.cc
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
  Server.cc
  Util.cc
)

set (gtest_sources
  Agent_TEST.cc
  Effector_TEST.cc
  GameState_TEST.cc
  Geometry_TEST.cc
  Robocup3dsPlugin_TEST.cc
  Perceptors_TEST.cc
  Server_TEST.cc
  Util_TEST.cc
)

# make below target specific?
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

foreach(f ${PARSER_SOURCE})
  list(APPEND parser_files ../deps/${f})
endforeach()

robocup3ds_add_library(${PROJECT_NAME_LOWER}Plugin ${sources} ${parser_files})

target_link_libraries(${PROJECT_NAME_LOWER}Plugin
  gazebo_common
  gazebo_physics
  gazebo_sensors
  ${GAZEBO_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
)

set_target_properties(${PROJECT_NAME_LOWER}Plugin PROPERTIES SOVERSION ${PROJECT_MAJOR_VERSION} VERSION ${PROJECT_VERSION_FULL})
install (TARGETS ${PROJECT_NAME_LOWER}Plugin DESTINATION ${LIB_INSTALL_DIR}/${PROJECT_NAME}/ COMPONENT shlib)

robocup3ds_build_tests(${gtest_sources})
