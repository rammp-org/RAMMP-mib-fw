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
#include "lwip/sockets.h"
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
std::function<void(std::span<const uint8_t>)> s_log_tee;
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
  if (in_write || !s_log_tee)
    return static_cast<ssize_t>(size);
  in_write = true;
  s_log_tee(std::span<const uint8_t>(static_cast<const uint8_t *>(data), size));
  in_write = false;
  return static_cast<ssize_t>(size);
}

} // namespace

ServiceLink::ServiceLink(const Config &config)
    : BaseComponent("ServiceLink", config.log_level)
    , config_(config)
    , ota_({.reject_same_version = false, .log_level = espp::Logger::Verbosity::INFO}) {
  transports_.reserve(3); // the TCP transport is added later, while other tasks read these
}

ServiceLink::~ServiceLink() {
  s_log_tee = nullptr;
  if (tcp_task_)
    tcp_task_->stop();
  tcp_close_client();
  if (tcp_listen_fd_ >= 0)
    ::close(tcp_listen_fd_);
}

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
  logger_.info("Service link up: {}{}| fw {} {}", usb_transport_ ? "WebUSB (J3) " : "",
               uart_transport_ ? "UART (USB-C) " : "", app->project_name, app->version);
  announce();
  return true;
}

// The USB-C's CH343 cannot show a WebUSB landing page, so the console says where the
// panel is: at boot, then every 30 s until a panel streams.
void ServiceLink::announce() {
  last_announce_us_ = esp_timer_get_time();
  if (config_.landing_page_url.empty())
    return;
  logger_.info("Control panel: https://{} (Chrome or Edge: Connect, Web Serial, this USB-C "
               "port)",
               config_.landing_page_url);
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
  t->ota = std::make_unique<espp::OtaService>(
      ota_, espp::OtaService::Config{.send = send, .log_level = espp::Logger::Verbosity::INFO});
  t->worker = std::make_unique<espp::DispatcherWorker>(espp::DispatcherWorker::Config{
      .send = send,
      .on_overflow = [ota = t->ota.get()]() { ota->on_rx_overflow(); },
      .task_config = {.name = "svc_" + t->name, .stack_size_bytes = 8 * 1024},
  });
  auto &worker = *t->worker;
  worker.register_module(*t->telemetry);
  // OTA (module 0), wrapped so the application makes the plant safe on BEGIN; the service
  // still owns the session state machine (and restarts after a good FINISH).
  worker.register_module(
      t->ota->module_id(),
      [this, ota = t->ota.get()](const espp::stream_frame::Frame &f) {
        if (!f.is_reply() && f.type == 0x01 /* BEGIN */ && config_.before_ota) {
          logger_.warn("OTA begin: putting the MIB in a safe state");
          config_.before_ota();
        }
        ota->handle(f);
        if (!f.is_reply() && f.type == 0x04 /* ABORT */ && config_.ota_aborted)
          config_.ota_aborted();
      },
      t->ota->module_info());
  const reply_fn reply = send;
  const bool uart = t->name == "uart";
  worker.register_module(
      kSystemModule,
      [this, reply, uart](const espp::stream_frame::Frame &f) { handle_system(f, reply, uart); },
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
  route_stdout();
  return true;
}

// Telemetry and log frames are fire-and-forget: write_vendor() waits up to 250 ms for
// FIFO room, which with no host reading would stall the producer. Room now, or drop.
void ServiceLink::usb_send_best_effort(std::span<const uint8_t> f) {
  if (usb_ && usb_->is_vendor_connected() && usb_->vendor_write_available() >= f.size())
    usb_->write_vendor(f);
}

// Logs as console-module LOG frames to the hosts that stream: an unsolicited stream
// confuses other hosts. Called from inside stdout writes, on any task: never blocks.
void ServiceLink::tee_log(std::span<const uint8_t> bytes) {
  if (usb_transport_ && usb_transport_->telemetry->streaming() && usb_->is_vendor_connected() &&
      usb_->vendor_write_available() >= bytes.size() + 13)
    usb_->write_vendor(espp::stream_frame::build_frame(true, kConsoleModule, kConsoleLog, bytes));
  auto *tcp = tcp_transport_.load();
  if (tcp && tcp->telemetry->streaming() && tcp_client_fd_ >= 0) {
    auto frame = espp::stream_frame::build_frame(true, kConsoleModule, kConsoleLog, bytes);
    std::lock_guard<std::mutex> lock(tcp_log_mutex_);
    if (tcp_log_bytes_ + frame.size() <= 16 * 1024) { // else drop: logging must not stall
      tcp_log_bytes_ += frame.size();
      tcp_log_.push_back(std::move(frame));
    }
  }
}

void ServiceLink::route_stdout() {
  if (stdout_routed_)
    return;
  stdout_routed_ = true;
  if (kConsoleUart >= 0) {
    char uart[16];
    std::snprintf(uart, sizeof(uart), "/dev/uart/%d", kConsoleUart);
    s_uart_fd = ::open(uart, O_WRONLY);
  }
  s_log_tee = [this](std::span<const uint8_t> bytes) { tee_log(bytes); };
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
#if defined(CONFIG_ESP_CONSOLE_UART_BAUDRATE)
  uart_default_baud_ = CONFIG_ESP_CONSOLE_UART_BAUDRATE;
#endif
  uart_baud_ = uart_default_baud_;

  auto &t = add_transport(
      "uart", [this](std::span<const uint8_t> f) { uart_send(f); },
      [port](std::span<const uint8_t> f) {
        size_t room = 0;
        if (uart_get_tx_buffer_free_size(port, &room) == ESP_OK && room >= f.size())
          uart_write_bytes(port, f.data(), f.size());
      });
  uart_transport_ = &t;
  uart_rx_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this, port, worker = t.worker.get()](std::mutex &,
                                                        std::condition_variable &) -> bool {
        uint8_t buf[256];
        const int n = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(20));
        const int64_t now = esp_timer_get_time();
        if (n > 0) {
          uart_last_rx_us_ = now;
          worker->push(std::span<const uint8_t>(buf, static_cast<size_t>(n)));
        }
        // A panel that went away (tab closed, cable pulled) leaves the console fast:
        // put it back for idf.py monitor and the like.
        if (uart_baud_ != uart_default_baud_ && now - uart_last_rx_us_ > 3'000'000) {
          set_uart_baud(uart_default_baud_);
          logger_.info("No panel on the UART for 3 s: console back to {} baud", uart_default_baud_);
        }
        if (!uart_transport_->telemetry->streaming() && now - last_announce_us_ > 30'000'000)
          announce();
        return false;
      },
      .task_config = {.name = "svc_uart_rx", .stack_size_bytes = 3 * 1024},
  });
  uart_rx_task_->start();
  return true;
}

void ServiceLink::set_uart_baud(uint32_t baud) {
  const auto port = static_cast<uart_port_t>(kConsoleUart);
  uart_wait_tx_done(port, pdMS_TO_TICKS(200)); // what was queued goes out at the old rate
  uart_set_baudrate(port, baud);
  uart_baud_ = baud;
  uart_last_rx_us_ = esp_timer_get_time();
}

void ServiceLink::uart_send(std::span<const uint8_t> f) {
  uart_write_bytes(static_cast<uart_port_t>(kConsoleUart), f.data(), f.size());
}

// ---- TCP ------------------------------------------------------------------------------
// Modelled on the PACE RACER's EthLink: one select() loop over the listen socket and the
// client; a newcomer replaces the client (a stale session from a closed tab must not lock
// the panel out). Frames go out whole under one mutex with a send timeout; a socket that
// cannot take a frame in time is shut down, since a partial frame would corrupt the stream.

bool ServiceLink::start_tcp() {
  if (config_.tcp_port == 0 || tcp_transport_)
    return false;
  tcp_listen_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (tcp_listen_fd_ < 0) {
    logger_.error("TCP socket() failed: {}", errno);
    return false;
  }
  int one = 1;
  ::setsockopt(tcp_listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(config_.tcp_port);
  if (::bind(tcp_listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
      ::listen(tcp_listen_fd_, 1) < 0) {
    logger_.error("TCP bind/listen on :{} failed: {}", config_.tcp_port, errno);
    ::close(tcp_listen_fd_);
    tcp_listen_fd_ = -1;
    return false;
  }
  auto &t = add_transport(
      "tcp", [this](std::span<const uint8_t> f) { tcp_send(f); },
      [this](std::span<const uint8_t> f) { tcp_send(f); });
  tcp_transport_ = &t;
  route_stdout();
  tcp_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &, std::condition_variable &) -> bool {
        tcp_serve();
        return false;
      },
      .task_config = {.name = "svc_tcp", .stack_size_bytes = 4 * 1024, .priority = 3},
  });
  tcp_task_->start();
  logger_.info("Service stream on tcp :{} (panel over Ethernet: tools/mib_panel/eth_bridge.py)",
               config_.tcp_port);
  return true;
}

void ServiceLink::tcp_serve() {
  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(tcp_listen_fd_, &rd);
  int maxfd = tcp_listen_fd_;
  const int client = tcp_client_fd_;
  if (client >= 0) {
    FD_SET(client, &rd);
    maxfd = std::max(maxfd, client);
  }
  timeval tv{.tv_sec = 0, .tv_usec = 20000};
  if (::select(maxfd + 1, &rd, nullptr, nullptr, &tv) > 0) {
    if (FD_ISSET(tcp_listen_fd_, &rd)) {
      sockaddr_in peer{};
      socklen_t len = sizeof(peer);
      const int fd = ::accept(tcp_listen_fd_, reinterpret_cast<sockaddr *>(&peer), &len);
      if (fd >= 0) {
        tcp_close_client();
        int one = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        timeval snd{.tv_sec = 0, .tv_usec = 300000};
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
        tcp_transport_.load()->worker->request_reset(); // a new byte stream
        tcp_client_fd_ = fd;
        char ip[16];
        inet_ntoa_r(peer.sin_addr, ip, sizeof(ip));
        logger_.info("Panel connected over TCP from {}", ip);
      }
    }
    if (client >= 0 && client == tcp_client_fd_ && FD_ISSET(client, &rd)) {
      uint8_t buf[512];
      const int n = ::recv(client, buf, sizeof(buf), 0);
      if (n > 0) {
        tcp_transport_.load()->worker->push(std::span<const uint8_t>(buf, static_cast<size_t>(n)));
      } else {
        logger_.info("Panel disconnected from TCP");
        tcp_close_client();
      }
    }
  }
  // Queued log frames, sent here so a log line never waits on the socket.
  std::deque<std::vector<uint8_t>> logs;
  {
    std::lock_guard<std::mutex> lock(tcp_log_mutex_);
    logs.swap(tcp_log_);
    tcp_log_bytes_ = 0;
  }
  for (const auto &f : logs)
    tcp_send(f);
}

void ServiceLink::tcp_send(std::span<const uint8_t> f) {
  std::lock_guard<std::mutex> lock(tcp_tx_mutex_);
  const int fd = tcp_client_fd_;
  if (fd < 0)
    return;
  size_t sent = 0;
  while (sent < f.size()) {
    const int n = ::send(fd, f.data() + sent, f.size() - sent, 0);
    if (n <= 0) {
      ::shutdown(fd, SHUT_RDWR); // the select loop sees EOF and closes the client
      return;
    }
    sent += static_cast<size_t>(n);
  }
}

void ServiceLink::tcp_close_client() {
  {
    std::lock_guard<std::mutex> lock(tcp_tx_mutex_);
    const int fd = tcp_client_fd_.exchange(-1);
    if (fd >= 0)
      ::close(fd);
  }
  if (auto *t = tcp_transport_.load())
    t->telemetry->set_streaming(false); // the next panel asks again
}

// ---- OTA ------------------------------------------------------------------------------

void ServiceLink::confirm_image_after(std::chrono::seconds delay) {
  if (!ota_.is_pending_verify())
    return;
  logger_.warn("Running a new image ({}), pending verify: confirming it in {} s unless a host "
               "does",
               ota_.running_partition_label(), delay.count());
  std::thread([this, delay] {
    std::this_thread::sleep_for(delay);
    if (!ota_.is_pending_verify())
      return; // a host already decided
    std::error_code ec;
    if (ota_.mark_app_valid(ec))
      logger_.info("New image confirmed after {} s of healthy running", delay.count());
    else
      logger_.error("Confirming the new image failed: {}", ec.message());
  }).detach();
}

// ---- telemetry ------------------------------------------------------------------------

void ServiceLink::emit(std::span<const float> values) {
  const uint32_t n = emit_count_.fetch_add(1);
  const auto now_us = static_cast<uint32_t>(esp_timer_get_time());
  if (usb_transport_ && usb_transport_->telemetry->streaming())
    usb_transport_->telemetry->emit(values, now_us);
  // At the console's 115200 baud the full rate does not fit beside the logs and status.
  const uint32_t divider =
      uart_baud_ >= 460800 ? 1 : static_cast<uint32_t>(std::max(1, config_.uart_telemetry_divider));
  if (uart_transport_ && uart_transport_->telemetry->streaming() && n % divider == 0)
    uart_transport_->telemetry->emit(values, now_us);
  if (auto *tcp = tcp_transport_.load(); tcp && tcp->telemetry->streaming())
    tcp->telemetry->emit(values, now_us);
}

bool ServiceLink::streaming() const {
  const auto *tcp = tcp_transport_.load();
  return (usb_transport_ && usb_transport_->telemetry->streaming()) ||
         (uart_transport_ && uart_transport_->telemetry->streaming()) ||
         (tcp && tcp->telemetry->streaming());
}

// ---- system module --------------------------------------------------------------------

void ServiceLink::handle_system(const espp::stream_frame::Frame &f, const reply_fn &reply,
                                bool uart) {
  if (f.is_reply())
    return;
  if (f.type == kSystemSetBaud) {
    if (!uart || f.payload.size() < 4)
      return;
    const uint32_t baud = espp::stream_frame::get_u32(f.payload);
    if (baud < 9600 || baud > 2'000'000)
      return;
    const uint8_t which = kSystemSetBaud;
    reply(espp::stream_frame::build_frame(true, kSystemModule, kSystemOk,
                                          std::span<const uint8_t>(&which, 1)));
    set_uart_baud(baud);
    return;
  }
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
        "\"idf\":\"{}\",\"reset_reason\":\"{}\",\"uptime_s\":{},\"usb\":{},\"uart\":{},\"tcp\":{},"
        "\"partition\":\"{}\","
        "\"pending_verify\":{}",
        config_.product, app->project_name, app->version, app->date, app->time, app->idf_ver,
        reset_reason_name(esp_reset_reason()), esp_timer_get_time() / 1000000,
        usb_transport_ ? "true" : "false", uart_transport_ ? "true" : "false",
        tcp_transport_.load() ? "true" : "false", ota_.running_partition_label(),
        ota_.is_pending_verify() ? "true" : "false");
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
