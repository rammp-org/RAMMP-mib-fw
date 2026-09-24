# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(ESPP_TRAJECTORY_PLANNER_DIR
  "${CMAKE_CURRENT_LIST_DIR}/../espp/components/trajectory_planner"
)
if(NOT EXISTS "${ESPP_TRAJECTORY_PLANNER_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "Missing local ESPP trajectory_planner component at ${ESPP_TRAJECTORY_PLANNER_DIR}. "
    "Clone ESPP at ../espp or replace this local path with espp/trajectory_planner "
    "after the component is published.")
endif()
message(WARNING
  "Using local ESPP trajectory_planner at ${ESPP_TRAJECTORY_PLANNER_DIR}; "
  "replace this path with espp/trajectory_planner after publication.")

set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/components"
  # TODO: replace with espp/trajectory_planner after it is published.
  "${ESPP_TRAJECTORY_PLANNER_DIR}"
)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/external/rammp-rtps/components")
  list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/external/rammp-rtps/components")
else()
  message(FATAL_ERROR "Missing external/rammp-rtps submodule. Run: git submodule update --init --recursive")
endif()

set(
  COMPONENTS
  "main esptool_py bsp MIBConfig MibSystem DifferentialDrive DriveController trajectory_planner rammp_rtps_messages"
  CACHE STRING
  "List of components to include"
  )

# Project metadata for the RAMMP MIB firmware
project(rammp_mib)

set(CMAKE_CXX_STANDARD 20)
