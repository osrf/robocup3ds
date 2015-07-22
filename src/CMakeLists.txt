
set(PARSER_SOURCE ../lib/sexpLibrary/parser.c ../lib/sexpLibrary/sexp.c ../lib/sexpLibrary/event_temp.c ../lib/sexpLibrary/sexp_ops.c ../lib/sexpLibrary/cstring.c ../lib/sexpLibrary/faststack.c)
Set(PARSER_HEADER ../lib/sexpLibrary/sexp.h ../lib/sexpLibrary/cstring.h ../lib/sexpLibrary/faststack.h ../lib/sexpLibrary/sexp_errors.h ../lib/sexpLibrary/sexp_ops.h )


set (sources
  Effector.cc
  Robocup3dsPlugin.cc
  Server.cc
)

set (gtest_sources
  Parser_TEST.cc
  Robocup3dsPlugin_TEST.cc
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

target_link_libraries(${PROJECT_NAME_LOWER}Plugin
  ${GAZEBO_LIBRARIES}
)

robocup3ds_install_library(${PROJECT_NAME_LOWER}Plugin)

robocup3ds_build_tests(${gtest_sources})
