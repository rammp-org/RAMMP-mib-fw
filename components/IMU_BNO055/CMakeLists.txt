   idf_component_register(
       SRCS "src/IMU_BNO055.cpp"
       INCLUDE_DIRS "include"
       REQUIRES driver espp__base_component   # whatever it depends on
   )