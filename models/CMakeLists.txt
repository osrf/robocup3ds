set (folders
  nao_blue
  nao_red
)

foreach (folder ${folders})
  install(DIRECTORY ${folder}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models)
endforeach()
