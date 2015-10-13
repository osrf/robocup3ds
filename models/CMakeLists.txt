set (folders
  nao_blue
  nao_red
  nao_type_zero	
  robocup_3Dsim_goal
  robocup_3Dsim_field
  robocup_3Dsim_ball
)

foreach (folder ${folders})
  install(DIRECTORY ${folder}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models)
endforeach()
