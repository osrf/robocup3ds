set(TEST_TYPE "INTEGRATION")

set(tests
  physics.cc
  run_agent_01.cc
  run_agent_02.cc
  run_agent_03.cc
  run_agent_04.cc
  run_agent_05.cc
)

# set_source_files_properties(run_agent.cc PROPERTIES COMPILE_FLAGS -v)

link_directories(${PROJECT_BINARY_DIR}/test)

robocup3ds_build_tests(${tests})
