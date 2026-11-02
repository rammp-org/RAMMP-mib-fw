idf_component_register(
  SRCS "src/drive_profile_store.cpp"
  INCLUDE_DIRS "include"
  REQUIRES base_component persistent_data file_system rammp_rtps_messages MIBConfig DriveController
)
