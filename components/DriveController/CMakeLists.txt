idf_component_register(
  SRCS "src/drive_controller.cpp"
  INCLUDE_DIRS "include"
  REQUIRES base_component timer DriveResponse DifferentialDrive rammp_rtps_messages MIBConfig
)
