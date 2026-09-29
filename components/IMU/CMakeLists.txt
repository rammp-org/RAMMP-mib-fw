# components/imu/CMakeLists.txt
idf_component_register(
  SRCS "src/imu.cpp"
  INCLUDE_DIRS "."
  REQUIRES adithya-187326__bno055 driver
  REQUIRES logger
)