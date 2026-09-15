#include "mib_system.hpp"

extern "C" void app_main(void) {
  MibSystem system;
  system.start();
}
