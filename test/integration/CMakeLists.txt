set(TEST_TYPE "INTEGRATION")

set(tests
  perceptor.cc
  physics.cc
  run_agent_01.cc
  run_agent_02.cc
  run_agent_03.cc
  run_agent_04.cc
  run_agent_05.cc
)

# set_source_files_properties(run_agent.cc PROPERTIES COMPILE_FLAGS -v)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PROJECT_BINARY_DIR}/test)

robocup3ds_build_tests(${tests})

add_library(perceptor_plugin SHARED perceptor_plugin.cc)
target_link_libraries(perceptor_plugin
  ${PROJECT_NAME_LOWER}Plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
  gtest gtest_main)
