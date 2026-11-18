#include "wire.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#elif CONFIG_ESP_CONSOLE_UART
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#else
#error "actuator_sysid needs the console on USB-Serial-JTAG or a UART"
#endif

namespace sysid {

namespace {

// Output is queued in the driver's TX buffer and drained by its interrupt, so a
// row costs the sample task a copy. 16 KB is ~0.3 s of the worst-case 57 kB/s.
constexpr int kTxBufferBytes = 16 * 1024;
constexpr int kRxBufferBytes = 1024;
// stdout's buffer, for the lines other code prints (espp's Logger): larger than
// any such line, so each reaches the driver in a single write.
constexpr size_t kStdoutBufferBytes = 2048;

std::atomic<uint32_t> g_dropped_lines{0};

#if CONFIG_ESP_CONSOLE_UART
constexpr uart_port_t kConsoleUart = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
#endif

/// Hand one complete line (newline included) to the console driver.
void write_line(const char *data, size_t size) {
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  // Never waits: with no host reading the port the TX buffer fills, and the
  // sample loop must keep its period (the motion guards run in it). The ring
  // buffer takes the whole line or none of it, so a full buffer drops whole
  // lines, never part of one; they are counted in #meta:tx_dropped_lines.
  if (usb_serial_jtag_write_bytes(data, size, 0) != static_cast<int>(size)) {
    g_dropped_lines++;
  }
#else
  // A UART drains at the baud rate whether or not anyone listens (the CH343 has
  // no flow control here), so the buffer only fills if the rows outrun the baud.
  // Never wait: write the line only if all of it fits now, else drop all of it
  // and count it. Every other stdout writer is held off by the stdout lock the
  // caller holds, so the free space cannot shrink before the write.
  size_t free_bytes = 0;
  if (uart_get_tx_buffer_free_size(kConsoleUart, &free_bytes) == ESP_OK && free_bytes >= size) {
    uart_write_bytes(kConsoleUart, data, size);
  } else {
    g_dropped_lines++;
  }
#endif
}

/// ESP-IDF log messages, reformatted as `#log,<text>`: colour codes and line
/// breaks removed, so a message is one protocol line however it was formatted.
int log_line(const char *format, va_list args) {
  char text[256];
  const int n = vsnprintf(text, sizeof(text), format, args);
  std::string line = "#log,";
  for (const char *p = text; *p != '\0'; ++p) {
    if (*p == '\033') { // skip an ANSI colour sequence: ESC [ ... m
      while (*p != '\0' && *p != 'm') {
        ++p;
      }
      if (*p == '\0') {
        break;
      }
      continue;
    }
    if (*p == '\n' || *p == '\r') {
      continue;
    }
    line += *p;
  }
  if (line.size() > 5) { // drop messages that were only a line break
    emit_line(line);
  }
  return n;
}

} // namespace

int64_t now_us() { return esp_timer_get_time(); }

uint32_t dropped_lines() { return g_dropped_lines.load(); }

void wire_init() {
  // Driver-backed console in both directions: reads come from the RX buffer
  // (read_console()), and stdout's own writes (espp's Logger) go through the same
  // driver as ours.
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  config.tx_buffer_size = kTxBufferBytes;
  config.rx_buffer_size = kRxBufferBytes;
  usb_serial_jtag_driver_install(&config);
  usb_serial_jtag_vfs_use_driver();
#else
  uart_driver_install(kConsoleUart, kRxBufferBytes, kTxBufferBytes, 0, nullptr, 0);
  uart_vfs_dev_use_driver(kConsoleUart);
#endif
  setvbuf(stdout, nullptr, _IOLBF, kStdoutBufferBytes);
  // End whatever partial line the boot left (ROM text at 115200 reads as noise at
  // 921600), so the first protocol line starts at the beginning of a line.
  write_line("\n", 1);
  esp_log_set_vprintf(log_line);
  esp_log_level_set("*", ESP_LOG_WARN);
}

void emit_line(std::string_view line) {
  std::string text(line);
  text += '\n';
  // stdout's lock is the one newlib takes for every printf / fwrite on stdout
  // (espp's Logger included). Holding it while our line goes to the driver means
  // no other task's output can land inside the line.
  flockfile(stdout);
  fflush(stdout); // anything printed to stdout before this line goes first
  write_line(text.data(), text.size());
  funlockfile(stdout);
}

void emit_event(std::string_view text) {
  std::string line = "#evt," + std::to_string(now_us()) + ",";
  for (char c : text) {
    line += (c == '\n' || c == '\r') ? ' ' : c;
  }
  emit_line(line);
}

void emit_meta(std::string_view key, std::string_view value) {
  std::string line = "#meta:";
  line.append(key);
  line += '=';
  line.append(value);
  emit_line(line);
}

int read_console(char *buffer, int size) {
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  const int n = usb_serial_jtag_read_bytes(buffer, size, 0);
#else
  const int n = uart_read_bytes(kConsoleUart, buffer, size, 0);
#endif
  return n > 0 ? n : 0;
}

} // namespace sysid
