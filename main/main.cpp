#include "mib_system.hpp"

extern "C" void app_main(void) {
  static MibSystem system; // static: its e-stop latch must stay off the main task stack
  system.start();
}
