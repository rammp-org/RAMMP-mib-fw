idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       REQUIRES bsp twai canopen mcp266 bno055 format esp_driver_uart esp_driver_usb_serial_jtag esp_timer
                                esp_app_format)
