find_package (Qt4)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

set (GUIplugins
  Robocup3dsGUIPlugin.cc
)

set (GUIpluginHeaders
  Robocup3dsGUIPlugin.hh
)

set (GUIpluginMOC
  moc_Robocup3dsGUIPlugin.cxx
)

# make below target specific?
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

# make below target specific?
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

QT4_WRAP_CPP(${GUIpluginMOC} ${GUIpluginHeaders})
robocup3ds_add_library(${PROJECT_NAME_LOWER}GUIPlugin ${GUIplugins} ${GUIpluginMOC} ${GUIpluginHeaders})

install (TARGETS ${PROJECT_NAME_LOWER}GUIPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_MAJOR_VERSION}/plugins/ COMPONENT shlib)

target_link_libraries(${PROJECT_NAME_LOWER}GUIPlugin
  ${GAZEBO_LIBRARIES}
  ${QT_LIBRARIES}
)
