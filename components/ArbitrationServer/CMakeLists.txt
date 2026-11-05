idf_component_register(SRCS "src/arbitration_server.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES Arbiter base_component rtps timer rammp_rtps_messages
                                rammp_rtps_services)
