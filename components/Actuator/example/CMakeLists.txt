# Bench example and test console for the Actuator component. Build from this
# directory with the same ESP-IDF as the firmware:
#
#   idf.py set-target esp32p4
#   idf.py build flash monitor
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# The repository's components directory, so Actuator and MIBConfig resolve. The
# espp components come through the component manager (main/idf_component.yml).
set(EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../.."
)

set(
  COMPONENTS
  "main esptool_py Actuator MIBConfig"
  CACHE STRING
  "List of components to include"
  )

project(actuator_example)

set(CMAKE_CXX_STANDARD 20)
