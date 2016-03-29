include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)
include (FindBoost)

########################################
# Find Gazebo
find_package(gazebo REQUIRED 7.0.0)

########################################
# Find Boost
find_package(Boost 1.54.0 COMPONENTS filesystem REQUIRED)
