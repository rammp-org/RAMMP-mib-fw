idf_component_register(SRCS "src/mib_system.cpp" "src/mib_panel.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES bsp rammp_rtps_messages DriveController DifferentialDrive RosMessages
                                estop_core ServiceLink esp_driver_tsens esp_app_format bno055)
