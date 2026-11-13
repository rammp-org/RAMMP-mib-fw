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

# Arbitration of the Jetson's TwistSetpoints (see arbitration_mode in MIBconfig.hpp).
# Default: disabled, as the arm driver's arbitration_mode parameter defaults.
#   idf.py -DMIB_ARBITRATION=enforced build
if(MIB_ARBITRATION STREQUAL "enforced")
  target_compile_definitions(${COMPONENT_LIB} INTERFACE MIB_ARBITRATION_ENFORCED)
elseif(MIB_ARBITRATION AND NOT MIB_ARBITRATION STREQUAL "disabled")
  message(FATAL_ERROR "Unknown MIB_ARBITRATION '${MIB_ARBITRATION}': use enforced or disabled")
endif()
