# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/components"
)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/external/rammp-rtps/components")
  list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/external/rammp-rtps/components")
else()
  message(FATAL_ERROR "Missing external/rammp-rtps submodule. Run: git submodule update --init --recursive")
endif()

set(
  COMPONENTS
  "main esptool_py bsp MibSystem rammp_rtps_messages"
  CACHE STRING
  "List of components to include"
  )

# Project metadata for the RAMMP MIB firmware
project(rammp_mib)

set(CMAKE_CXX_STANDARD 20)
