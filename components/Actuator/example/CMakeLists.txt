# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# The repo components the console needs, one by one: the BSP (which owns the CAN bus, the
# heartbeat and one Actuator per leg in MIBconfig.hpp) and what it requires. Pointing at the
# whole components/ directory would also pull in MibSystem, which needs the rammp-rtps
# submodule.
set(REPO_COMPONENTS "${CMAKE_CURRENT_LIST_DIR}/../..")
set(EXTRA_COMPONENT_DIRS
  "${REPO_COMPONENTS}/Actuator"
  "${REPO_COMPONENTS}/bsp"
  "${REPO_COMPONENTS}/MIBConfig"
  "${REPO_COMPONENTS}/Arbiter"
)

set(
  COMPONENTS
  "main esptool_py bsp Actuator"
  CACHE STRING
  "List of components to include"
  )

project(actuator_example)

set(CMAKE_CXX_STANDARD 20)
