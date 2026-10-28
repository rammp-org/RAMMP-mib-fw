#include "service_link.hpp"

#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs.h"
#include "sdkconfig.h"

using namespace std::chrono_literals;

namespace mib {

namespace {

#if defined(CONFIG_ESP_CONSOLE_UART_NUM) && CONFIG_ESP_CONSOLE_UART_NUM >= 0
constexpr int kConsoleUart = CONFIG_ESP_CONSOLE_UART_NUM;
#else
constexpr int kConsoleUart = -1;
#endif

const char *reset_reason_name(esp_reset_reason_t reason) {
  switch (reason) {
  case ESP_RST_POWERON:
    return "power-on";
  case ESP_RST_EXT:
    return "external";
  case ESP_RST_SW:
    return "software";
  case ESP_RST_PANIC:
    return "panic";
  case ESP_RST_INT_WDT:
    return "interrupt watchdog";
  case ESP_RST_TASK_WDT:
    return "task watchdog";
  case ESP_RST_WDT:
    return "watchdog";
  case ESP_RST_BROWNOUT:
    return "brownout";
  default:
    return "other";
  }
}

// ---- stdout tee: the console UART as before, plus console-module LOG frames on USB ----
// Modelled on DemoLink's VFS device. A write made by the USB stack from inside a write
// would recurse, so each task breaks the loop with a thread-local flag.
constexpr char kVfsPath[] = "/dev/mibcon";
std::function<void(std::span<const uint8_t>)> s_usb_log;
int s_uart_fd = -1;

int vfs_open(const char *, int, int) { return 0; }
int vfs_close(int) { return 0; }
int vfs_fstat(int, struct stat *st) {
  *st = {};
  st->st_mode = S_IFCHR;
  return 0;
}
ssize_t vfs_write(int, const void *data, size_t size) {
  static thread_local bool in_write = false;
  if (s_uart_fd >= 0)
    ::write(s_uart_fd, data, size);
  if (in_write || !s_usb_log)
    return static_cast<ssize_t>(size);
  in_write = true;
  s_usb_log(std::span<const uint8_t>(static_cast<const uint8_t *>(data), size));
  in_write = false;
  return static_cast<ssize_t>(size);
}

} // namespace

ServiceLink::ServiceLink(const Config &config)
    : BaseComponent("ServiceLink", config.log_level)
    , config_(config) {}

ServiceLink::~ServiceLink() { s_usb_log = nullptr; }

void ServiceLink::register_module(uint8_t module_id, handler_fn handler, std::string name,
                                  std::string description) {
  modules_.push_back({module_id, std::move(handler), std::move(name), std::move(description)});
}

bool ServiceLink::start() {
  bool any = false;
  if (config_.uart)
    any |= start_uart();
  if (config_.usb)
    any |= start_usb();
  if (!any) {
    logger_.error("No service transport came up");
    return false;
  }
  const auto *app = esp_app_get_description();
  logger_.info("Service link up: {}{}{} | fw {} {}", usb_transport_ ? "WebUSB (J3) " : "",
               uart_transport_ ? "UART (USB-C) " : "",
               usb_transport_ ? fmt::format("| landing https://{}", config_.landing_page_url) : "",
               app->project_name, app->version);
  return true;
}

ServiceLink::Transport &ServiceLink::add_transport(std::string name, reply_fn send,
                                                   reply_fn send_best_effort) {
  auto t = std::make_unique<Transport>();
  t->name = std::move(name);
  t->send = send;
  t->telemetry = std::make_unique<espp::Telemetry>(espp::Telemetry::Config{
      .channels = config_.channels,
      .send = send_best_effort,
      // Quiet until a host asks (SET_STREAM): nothing binary reaches the UART console
      // unless a panel opened it.
      .stream_on_start = false,
      .period_ms = static_cast<uint16_t>(20),
  });
  t->worker = std::make_unique<espp::DispatcherWorker>(espp::DispatcherWorker::Config{
      .send = send,
      .task_config = {.name = "svc_" + t->name, .stack_size_bytes = 6 * 1024},
  });
  auto &worker = *t->worker;
  worker.register_module(*t->telemetry);
  const reply_fn reply = send;
  worker.register_module(
      kSystemModule, [this, reply](const espp::stream_frame::Frame &f) { handle_system(f, reply); },
      {.name = "System", .app = "", .description = "Reset and device info", .protocol = ""});
  for (const auto &m : modules_) {
    auto handler = m.handler;
    worker.register_module(
        m.id, [handler, reply](const espp::stream_frame::Frame &f) { handler(f, reply); },
        {.name = m.name, .app = "", .description = m.description, .protocol = ""});
  }
  worker.serve_discovery(config_.product, config_.firmware);
  transports_.push_back(std::move(t));
  return *transports_.back();
}

// ---- USB ------------------------------------------------------------------------------

bool ServiceLink::start_usb() {
  espp::UsbDevice::Config u;
  u.pid = 0x0d42; // VID 0x1209 (pid.codes, the espp default); the PACE RACER uses 0x0d41
  u.manufacturer = "RAMMP";
  u.product = config_.product;
  u.serial_number = "MIB-0001";
  u.port = 1; // the P4's USB 2.0 HS controller: J3 on the ESP32-P4-ETH
  u.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = config_.product + " (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = config_.landing_page_url;
  u.vendor = vendor;
  usb_ = std::make_unique<espp::UsbDevice>(u);

  auto &t = add_transport(
      "usb", [this](std::span<const uint8_t> f) { usb_->write_vendor(f); },
      [this](std::span<const uint8_t> f) { usb_send_best_effort(f); });
  usb_transport_ = &t;
  usb_->set_vendor_receive_callback(
      [worker = t.worker.get()](std::span<const uint8_t> d) { worker->push(d); });

  std::error_code ec;
  if (!usb_->initialize(ec)) {
    logger_.error("USB device failed to initialize: {}", ec.message());
    usb_transport_ = nullptr;
    transports_.pop_back();
    usb_.reset();
    return false;
  }
  route_stdout_to_usb();
  return true;
}

// Telemetry and log frames are fire-and-forget: write_vendor() waits up to 250 ms for
// FIFO room, which with no host reading would stall the producer. Room now, or drop.
void ServiceLink::usb_send_best_effort(std::span<const uint8_t> f) {
  if (usb_ && usb_->is_vendor_connected() && usb_->vendor_write_available() >= f.size())
    usb_->write_vendor(f);
}

void ServiceLink::route_stdout_to_usb() {
  if (kConsoleUart >= 0) {
    char uart[16];
    std::snprintf(uart, sizeof(uart), "/dev/uart/%d", kConsoleUart);
    s_uart_fd = ::open(uart, O_WRONLY);
  }
  s_usb_log = [this](std::span<const uint8_t> bytes) {
    // Only while a panel streams telemetry: an unsolicited stream confuses other hosts.
    if (!usb_transport_ || !usb_transport_->telemetry->streaming())
      return;
    if (usb_->is_vendor_connected() && usb_->vendor_write_available() >= bytes.size() + 13)
      usb_->write_vendor(espp::stream_frame::build_frame(true, kConsoleModule, kConsoleLog, bytes));
  };
  esp_vfs_t vfs = {};
  vfs.flags = ESP_VFS_FLAG_DEFAULT;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  vfs.open = &vfs_open;
  vfs.write = &vfs_write;
  vfs.close = &vfs_close;
  vfs.fstat = &vfs_fstat;
#pragma GCC diagnostic pop
  if (esp_vfs_register(kVfsPath, &vfs, nullptr) != ESP_OK) {
    logger_.error("Could not register the console tee; logs stay on the UART only");
    return;
  }
  fflush(stdout);
  if (freopen(kVfsPath, "w", stdout) == nullptr) {
    esp_vfs_unregister(kVfsPath);
    return;
  }
  setvbuf(stdout, nullptr, _IONBF, 0);
}

// ---- UART -----------------------------------------------------------------------------

bool ServiceLink::start_uart() {
  if (kConsoleUart < 0) {
    logger_.warn("No UART console in this build; the UART service link is off");
    return false;
  }
  const auto port = static_cast<uart_port_t>(kConsoleUart);
  if (!uart_is_driver_installed(port)) {
    if (uart_driver_install(port, 2048, 4096, 0, nullptr, 0) != ESP_OK) {
      logger_.error("UART{} driver install failed", kConsoleUart);
      return false;
    }
  }
  // Every stdout write now goes through the driver's TX mutex, so a frame written in
  // one uart_write_bytes() call cannot be split by a log line.
  uart_vfs_dev_use_driver(kConsoleUart);

  auto &t = add_transport(
      "uart", [this](std::span<const uint8_t> f) { uart_send(f); },
      [port](std::span<const uint8_t> f) {
        size_t room = 0;
        if (uart_get_tx_buffer_free_size(port, &room) == ESP_OK && room >= f.size())
          uart_write_bytes(port, f.data(), f.size());
      });
  uart_transport_ = &t;
  uart_rx_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [port, worker = t.worker.get()](std::mutex &, std::condition_variable &) -> bool {
        uint8_t buf[256];
        const int n = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0)
          worker->push(std::span<const uint8_t>(buf, static_cast<size_t>(n)));
        return false;
      },
      .task_config = {.name = "svc_uart_rx", .stack_size_bytes = 3 * 1024},
  });
  uart_rx_task_->start();
  return true;
}

void ServiceLink::uart_send(std::span<const uint8_t> f) {
  uart_write_bytes(static_cast<uart_port_t>(kConsoleUart), f.data(), f.size());
}

// ---- telemetry ------------------------------------------------------------------------

void ServiceLink::emit(std::span<const float> values) {
  const uint32_t n = emit_count_.fetch_add(1);
  const auto now_us = static_cast<uint32_t>(esp_timer_get_time());
  if (usb_transport_ && usb_transport_->telemetry->streaming())
    usb_transport_->telemetry->emit(values, now_us);
  if (uart_transport_ && uart_transport_->telemetry->streaming() &&
      n % static_cast<uint32_t>(std::max(1, config_.uart_telemetry_divider)) == 0)
    uart_transport_->telemetry->emit(values, now_us);
}

bool ServiceLink::streaming() const {
  for (const auto &t : transports_)
    if (t->telemetry->streaming())
      return true;
  return false;
}

// ---- system module --------------------------------------------------------------------

void ServiceLink::handle_system(const espp::stream_frame::Frame &f, const reply_fn &reply) {
  if (f.is_reply())
    return;
  if (f.type == kSystemReset) {
    const uint8_t which = kSystemReset;
    reply(espp::stream_frame::build_frame(true, kSystemModule, kSystemOk,
                                          std::span<const uint8_t>(&which, 1)));
    logger_.warn("Reset requested over the service link");
    std::thread([] {
      std::this_thread::sleep_for(300ms);
      esp_restart();
    }).detach();
  } else if (f.type == kSystemInfo) {
    const auto *app = esp_app_get_description();
    std::string json = fmt::format(
        "{{\"product\":\"{}\",\"project\":\"{}\",\"version\":\"{}\",\"built\":\"{} {}\","
        "\"idf\":\"{}\",\"reset_reason\":\"{}\",\"uptime_s\":{},\"usb\":{},\"uart\":{}",
        config_.product, app->project_name, app->version, app->date, app->time, app->idf_ver,
        reset_reason_name(esp_reset_reason()), esp_timer_get_time() / 1000000,
        usb_transport_ ? "true" : "false", uart_transport_ ? "true" : "false");
    if (info_extra_) {
      const auto extra = info_extra_();
      if (!extra.empty())
        json += "," + extra;
    }
    json += "}";
    reply(espp::stream_frame::build_frame(
        true, kSystemModule, kSystemInfoReply,
        std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(json.data()), json.size())));
  }
}

} // namespace mib
