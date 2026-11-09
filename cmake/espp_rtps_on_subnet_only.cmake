# Builds espp/rtps's SPDPAgent.cpp without the fallback that adopts a peer with no address
# on the MIB's subnet.
#
# A Jetson node bound to another NIC announces only that NIC's address. The fallback adopts
# it anyway, every send to it fails, and each failure logs a line to the 115200 console from
# the send path. On the flatbot that gapped the wheel commands past the boards' 200 ms
# watchdog (rammp-org/RAMMP-mib-fw#75). The MIB's address is always known before its
# participant starts, so the case the fallback was written for can't occur here.
#
# The source is rewritten at configure time, so managed_components stays untouched, and the
# build stops if a new espp/rtps changes this code. Delete this file once espp/rtps has an
# option for it.
idf_component_get_property(rtps_lib espp__rtps COMPONENT_LIB)
idf_component_get_property(rtps_dir espp__rtps COMPONENT_DIR)
set(spdp_src "${rtps_dir}/src/discovery/SPDPAgent.cpp")
set(spdp_out "${CMAKE_BINARY_DIR}/espp_rtps/SPDPAgent.cpp")

file(READ "${spdp_src}" spdp_text)
set(fallback_regex
    "  // Fallback: if subnet check fails.*still use any valid unicast locator so SEDP matching can proceed\\.\n  if \\(!locator\\) {\n    for [^\n]*\n      LocatorIPv4 \\*l = [^\n]*\n      if \\(l->isValid\\(\\)\\) {\n        locator = l;\n        break;\n      }\n    }\n  }\n")
string(REGEX REPLACE "${fallback_regex}" "" spdp_patched "${spdp_text}")
if(spdp_patched STREQUAL spdp_text)
  message(FATAL_ERROR "espp/rtps: the off-subnet fallback in SPDPAgent.cpp has changed; "
                      "review cmake/espp_rtps_on_subnet_only.cmake")
endif()
file(CONFIGURE OUTPUT "${spdp_out}" CONTENT "${spdp_patched}" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${spdp_src}")

get_target_property(rtps_sources ${rtps_lib} SOURCES)
list(TRANSFORM rtps_sources REPLACE ".*/src/discovery/SPDPAgent\\.cpp$" "${spdp_out}")
if(NOT "${spdp_out}" IN_LIST rtps_sources)
  message(FATAL_ERROR "espp/rtps: SPDPAgent.cpp isn't among the component's sources")
endif()
set_target_properties(${rtps_lib} PROPERTIES SOURCES "${rtps_sources}")
