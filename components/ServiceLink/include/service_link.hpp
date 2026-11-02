#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "dispatcher_worker.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "stream_frame.hpp"
#include "task.hpp"
#include "telemetry.hpp"
#include "usb_device.hpp"

namespace mib {

/// @brief The MIB's service link: one framed protocol (espp stream_frame, routed by an
/// espp::DispatcherWorker) served on up to three transports at once.
///
/// - TCP on the MIB's Ethernet (port 5555 by default), one client at a time: the same
///   service stream as the PACE RACER's EthLink (atos-drive/main/eth_link.hpp). A browser
///   reaches it through tools/mib_panel/eth_bridge.py, which relays a WebSocket to it.
/// - WebUSB on the ESP32-P4's USB 2.0 HS port. Chrome offers the landing page (the
///   control panel) when it is plugged in. On the ESP32-P4-ETH this port is J3, the
///   4-pin header; the board's USB-C is a CH343 USB-UART bridge, not native USB.
/// - The console UART (UART0, the USB-C through the CH343), for Web Serial. Frames are
///   written between log lines; the panel separates the two by the frame magic and CRC.
///
/// Both transports carry the same modules:
///   0    OTA        espp::OtaService: firmware update, mark valid, roll back (one per
///                   transport on one espp::Ota engine; each owns only the session it began)
///   3    telemetry  espp::Telemetry: named float channels, device timestamps
///   0x10 console    device -> host stdout (USB and TCP; on the UART the logs are the stream)
///   0x20 system     reset, info
///   0xFF discovery  what the device serves
/// plus whatever the application adds with register_module().
///
/// Modelled on the PACE RACER's DemoLink and EthLink (pace-racer-fw atos-drive/main/), without
/// the core dump.
class ServiceLink : public espp::BaseComponent {
public:
  /// Sends one encoded frame back on the transport a request arrived on.
  using reply_fn = std::function<void(std::span<const uint8_t>)>;
  /// An application module: the frame, and how to answer on its transport.
  using handler_fn = std::function<void(const espp::stream_frame::Frame &, const reply_fn &)>;

  static constexpr uint8_t kConsoleModule = 0x10;
  static constexpr uint8_t kConsoleLog = 0x82; ///< device -> host: UTF-8 stdout chunk
  static constexpr uint8_t kSystemModule = 0x20;
  static constexpr uint8_t kSystemReset = 0x01; ///< host -> device: reboot
  static constexpr uint8_t kSystemInfo = 0x02;  ///< host -> device: JSON info
  /// host -> device, UART only: [baud u32]. Replies OK at the old rate, then switches.
  /// The UART returns to its default rate 3 s after the last byte from the host.
  static constexpr uint8_t kSystemSetBaud = 0x03;
  static constexpr uint8_t kSystemOk = 0x81;        ///< device -> host: [u8 request type]
  static constexpr uint8_t kSystemInfoReply = 0x82; ///< device -> host: UTF-8 JSON

  struct Config {
    std::string product{"RAMMP MIB"};  ///< USB product string and discovery name.
    std::string firmware{""};          ///< Reported by discovery and info.
    std::vector<std::string> channels; ///< Telemetry channel names, in emit() order.
    bool usb{true};                    ///< Serve WebUSB on the HS port.
    bool uart{true};                   ///< Serve on the console UART.
    std::string landing_page_url{""};  ///< WebUSB landing page, without a scheme.
    int uart_telemetry_divider{5};     ///< Emit every Nth sample on the UART.
    uint16_t tcp_port{5555};           ///< TCP service port; 0 disables. See start_tcp().
    /// Called (on a service-link worker task) right before an OTA session begins, so the
    /// application can put the plant in a safe state: flash writes stall the cache.
    std::function<void()> before_ota{nullptr};
    /// Called when an OTA session is aborted (the update will not happen).
    std::function<void()> ota_aborted{nullptr};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  explicit ServiceLink(const Config &config);
  ~ServiceLink();

  ServiceLink(const ServiceLink &) = delete;
  ServiceLink &operator=(const ServiceLink &) = delete;

  /// Bring the transports up and register the built-in modules. Call once, after every
  /// register_module(). Returns false if no transport came up.
  bool start();

  /// Open the TCP service port. Call once the TCP/IP stack is up (after the BSP brings
  /// Ethernet up); a link or lease is not needed to listen. Call after start().
  bool start_tcp();

  /// Serve `module_id` on every transport. Handlers run on that transport's worker task.
  void register_module(uint8_t module_id, handler_fn handler, std::string name,
                       std::string description);

  /// One telemetry sample, values in Config::channels order. Best-effort: a transport
  /// with no host streaming, or no room in its FIFO, drops it.
  void emit(std::span<const float> values);

  /// True while some host has asked for telemetry.
  bool streaming() const;

  /// A new image boots PENDING_VERIFY (rollback is on); this confirms it after `delay`, if no
  /// host has. Call once the application is healthy (here: RTPS up). An image that never gets
  /// there stays pending, and the bootloader boots the previous one at the next reset.
  void confirm_image_after(std::chrono::seconds delay);

  /// Extra fields for the system module's info reply, as `"key":value` JSON members.
  void set_info_extra(std::function<std::string()> extra) { info_extra_ = std::move(extra); }

private:
  struct Transport {
    std::string name;
    reply_fn send;
    std::unique_ptr<espp::Telemetry> telemetry;
    std::unique_ptr<espp::OtaService> ota;
    std::unique_ptr<espp::DispatcherWorker> worker;
  };

  Transport &add_transport(std::string name, reply_fn send, reply_fn send_best_effort);
  void handle_system(const espp::stream_frame::Frame &frame, const reply_fn &reply, bool uart);
  void set_uart_baud(uint32_t baud);
  void announce();
  bool start_usb();
  bool start_uart();
  void route_stdout();
  void tee_log(std::span<const uint8_t> bytes);
  void tcp_serve();
  void tcp_send(std::span<const uint8_t> frame);
  void tcp_close_client();
  void uart_send(std::span<const uint8_t> frame);
  void usb_send_best_effort(std::span<const uint8_t> frame);

  Config config_;
  espp::Ota ota_; ///< The one OTA engine every transport's OtaService shares.
  std::unique_ptr<espp::UsbDevice> usb_;
  std::vector<std::unique_ptr<Transport>> transports_;
  struct Module {
    uint8_t id;
    handler_fn handler;
    std::string name;
    std::string description;
  };
  std::vector<Module> modules_;
  std::function<std::string()> info_extra_;
  std::unique_ptr<espp::Task> uart_rx_task_;
  std::atomic<uint32_t> emit_count_{0};
  uint32_t uart_default_baud_{115200};      ///< The console rate, from sdkconfig.
  std::atomic<uint32_t> uart_baud_{115200}; ///< The rate in force.
  std::atomic<int64_t> uart_last_rx_us_{0}; ///< Last byte from a host on the UART.
  int64_t last_announce_us_{0};             ///< Last console line with the panel URL.
  Transport *usb_transport_{nullptr};
  Transport *uart_transport_{nullptr};
  std::atomic<Transport *> tcp_transport_{nullptr};
  bool stdout_routed_{false};
  int tcp_listen_fd_{-1};
  std::atomic<int> tcp_client_fd_{-1};
  std::mutex tcp_tx_mutex_;                  ///< One frame at a time onto the socket.
  std::mutex tcp_log_mutex_;                 ///< Guards tcp_log_.
  std::deque<std::vector<uint8_t>> tcp_log_; ///< LOG frames for the TCP task to send.
  size_t tcp_log_bytes_{0};
  std::unique_ptr<espp::Task> tcp_task_;
};

} // namespace mib
