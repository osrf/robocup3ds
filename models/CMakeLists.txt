set (folders
  nao_blue
  nao_red
  nao_type_zero
)

foreach (folder ${folders})
  install(DIRECTORY ${folder}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/models)
endforeach()
