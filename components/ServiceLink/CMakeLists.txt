idf_component_register(
  SRCS "src/service_link.cpp"
  INCLUDE_DIRS "include"
  REQUIRES base_component logger task usb_device dispatcher stream_frame telemetry
           esp_driver_uart vfs esp_timer esp_app_format
)
