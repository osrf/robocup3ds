set (files
  robocup3d.world
)

install(FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/worlds)
