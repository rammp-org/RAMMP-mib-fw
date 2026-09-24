idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES
    "base_component"
    "esp_driver_gpio"
    "ethernet"
    "esp_netif"
    "esp32-p4-eth"
    "rtps"
    "cdr"
    "logger"
    "MIBConfig"
  REQUIRED_IDF_TARGETS "esp32p4"
)
