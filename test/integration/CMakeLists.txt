set(TEST_TYPE "INTEGRATION")

set(tests
  physics.cc
  run_agent.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

robocup3ds_build_tests(${tests})
