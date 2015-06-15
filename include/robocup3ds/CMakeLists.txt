include (${project_cmake_dir}/Utils.cmake)

set (headers
  Robocup3dsPlugin.hh
 #  states/BeforeKickOffState.hh
	# states/CornerKickLeftState.hh
	# states/CornerKickRightState.hh
	# states/FreeKickLeftState.hh
	# states/FreeKickRightState.hh
	# states/GameOverState.hh
	# states/GoalKickLeftState.hh
	# states/GoalKickRightState.hh
	# states/GoalLeftState.hh
	# states/GoalRightState.hh
	# states/KickInLeftState.hh
	# states/KickInRightState.hh
	# states/KickOffLeftState.hh
	# states/KickOffRightState.hh
	# states/PlayState.hh
)

set (robocup3ds_headers "" CACHE INTERNAL "RoboCup 3DS headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(robocup3ds_headers
    "RoboCup3DS headers" "#include <robocup3ds/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/robocup3ds_auto_headers.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/robocup3ds.hh)

robocup3ds_install_includes("" ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hh)
robocup3ds_install_includes(${PROJECT_NAME} ${headers})
