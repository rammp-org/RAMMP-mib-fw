idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES esp_driver_gpio Arbiter
)

# Robot selection (see differential_drive in MIBconfig.hpp). Default: the chair.
#   idf.py -DMIB_ROBOT=flatbot build
if(MIB_ROBOT STREQUAL "flatbot")
  target_compile_definitions(${COMPONENT_LIB} INTERFACE MIB_ROBOT_FLATBOT)
elseif(MIB_ROBOT AND NOT MIB_ROBOT STREQUAL "chair")
  message(FATAL_ERROR "Unknown MIB_ROBOT '${MIB_ROBOT}': use chair or flatbot")
endif()
