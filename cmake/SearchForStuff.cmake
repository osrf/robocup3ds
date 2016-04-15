include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)
include (FindBoost)

########################################
# Find Gazebo
find_package(gazebo REQUIRED 7.0.0)
if (NOT gazebo_FOUND)
  message(STATUS "Looking for gazebo 7 - not found")
  BUILD_ERROR ("Missing gazebo package. Please install libgazebo7-dev")
else()
  message(STATUS "Looking for gazebo  - ${GAZEBO_VERSION} found")
endif()


########################################
# Find Boost
find_package(Boost 1.54.0
             COMPONENTS system filesystem thread regex iostreams REQUIRED)
if (NOT Boost_FOUND)
  BUILD_ERROR ("Boost not found. Please install thread system filesystem regex iostreams boost version 1.54.0 or higher.")
endif()
