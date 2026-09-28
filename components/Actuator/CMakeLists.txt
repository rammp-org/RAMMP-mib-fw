idf_component_register(
  SRCS "src/actuator.cpp" "src/actuator_store.cpp"
  INCLUDE_DIRS "include"
  REQUIRES base_component mcp266 canopen nvs JointModel
)
