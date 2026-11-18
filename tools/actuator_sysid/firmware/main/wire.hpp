#pragma once

/// The console data channel: one writer for every line this firmware prints.
///
/// Line types (README, "Wire format"):
///   #cols:<name>,<name>,...   column names of the D rows
///   D,<v>,<v>,...             one sample; an empty field means no data
///   #meta:<key>=<value>       metadata
///   #evt,<t_us>,<text>        an event; <text> runs to the end of the line
///   #log,<text>               an ESP-IDF log message, routed here so it stays one line
/// Anything else (the ROM's boot lines, an espp component's ERROR line) is not part
/// of the protocol, and always arrives as whole lines between rows.

#include <cstdint>
#include <string>
#include <string_view>

namespace sysid {

/// Microseconds since boot (esp_timer). The time base of t_us and of every event.
int64_t now_us();

/// Install the console driver (UART0 through the CH343 by default; USB-Serial-JTAG
/// is supported if the console is moved there), make stdout line-buffered, route ESP-IDF logging to `#log,` lines, and
/// lower the ESP-IDF log level to WARN. Call once, before any other task prints.
void wire_init();

/// Print one line (a newline is appended). The whole line goes out in one write
/// under stdout's lock, so lines from different tasks never interleave. Never
/// waits: a line that does not fit the TX buffer is dropped whole and counted.
void emit_line(std::string_view line);

/// Lines dropped because the TX buffer was full.
uint32_t dropped_lines();

/// Print `#evt,<now_us>,<text>`. Newlines in text become spaces.
void emit_event(std::string_view text);

/// Print `#meta:<key>=<value>`.
void emit_meta(std::string_view key, std::string_view value);

/// Read up to `size` bytes from the console without waiting. Returns the count.
int read_console(char *buffer, int size);

} // namespace sysid
