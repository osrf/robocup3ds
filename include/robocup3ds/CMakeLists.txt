include (${project_cmake_dir}/Utils.cmake)

set (headers
  Robocup3dsPlugin.hh
  SocketParser.hh
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
