set(TEST_TYPE "INTEGRATION")

set(tests
  physics.cc
  # run_agent.cc
)

# set_source_files_properties(run_agent.cc PROPERTIES COMPILE_FLAGS -v)

link_directories(${PROJECT_BINARY_DIR}/test)

robocup3ds_build_tests(${tests})
