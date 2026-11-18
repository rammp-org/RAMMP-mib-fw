# Standalone MIB firmware that logs the IMU and the MCP266 actuator controllers for
# system identification. See README.md. Not part of the main firmware build.
#
# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Only the repo components the bsp needs, listed one by one: pointing at the whole
# components/ directory would also pull in MibSystem and friends, which need the
# rammp-rtps submodule.
set(REPO_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
set(EXTRA_COMPONENT_DIRS
  "${REPO_COMPONENTS}/bsp"
  "${REPO_COMPONENTS}/MIBConfig"
  "${REPO_COMPONENTS}/Arbiter"
)

set(
  COMPONENTS
  "main esptool_py bsp"
  CACHE STRING
  "List of components to include"
  )

project(actuator_sysid)

set(CMAKE_CXX_STANDARD 20)
