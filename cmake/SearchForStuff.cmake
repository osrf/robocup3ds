include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)
include (FindBoost)

########################################
# Find SDFormat
set (SDFormat_MIN_VERSION 3.0.3)
find_package(SDFormat ${SDFormat_MIN_VERSION})

if (NOT SDFormat_FOUND)
  message (STATUS "Looking for SDFormat - not found")
  BUILD_ERROR ("Missing: SDF version >=${SDFormat_MIN_VERSION}. Required for reading and writing SDF files.")
else()
  message (STATUS "Looking for SDFormat - found")
endif()

find_package(gazebo REQUIRED)
find_package(Boost 1.54.0 COMPONENTS system filesystem thread REQUIRED)

# for haptix GUI plugin
find_package(Qt4)
