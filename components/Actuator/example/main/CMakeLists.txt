idf_component_register(
  SRCS "actuator_example.cpp"
  INCLUDE_DIRS "."
  REQUIRES Actuator MIBConfig twai canopen mcp266 cli
)
