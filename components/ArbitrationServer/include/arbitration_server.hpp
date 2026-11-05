#pragma once

#include <chrono>
#include <memory>
#include <mutex>

#include "arbiter.hpp"
#include "base_component.hpp"
#include "messages/control_message.hpp"
#include "rtps_pubsub.hpp"
#include "rtps_service.hpp"
#include "services/control_service.hpp"
#include "timer.hpp"

namespace mib {

/// @brief The Arbiter's face on the wire: the control services and the status topic.
///
/// The base's half of the protocol the arm serves from kinova-gen3-ros2, with the same
/// ROS 2 types, under the "base" namespace (see rammp-rtps
/// messages/control_message.hpp):
///
/// | Endpoint                | Type                                         |
/// |-------------------------|----------------------------------------------|
/// | /base/acquire_control   | rammp_common_interfaces/srv/AcquireControl   |
/// | /base/release_control   | rammp_common_interfaces/srv/ReleaseControl   |
/// | /base/revoke_control    | rammp_common_interfaces/srv/RevokeControl    |
/// | /base/control_status    | rammp_common_interfaces/msg/ControlStatus    |
///
/// The services call straight into the Arbiter. The status is checked every
/// `poll_period` and published when it changes, as the arm's is, and also resent every
/// `resend_period` whether it changed or not, as every MIB status is.
///
/// Not here: the tokened commands, which belong to whoever acts on them (each handler
/// wraps its work in Arbiter::run_if_admitted()), and the e-stop, which on the MIB is
/// the estop-core latch rather than the arm's /estop topic.
class ArbitrationServer : public espp::BaseComponent {
public:
  /// @brief ArbitrationServer configuration.
  struct Config {
    Arbiter &arbiter;                           ///< Who may command the base. Must outlive this.
    espp::RtpsParticipant &participant;         ///< Started, with RPC enabled. Must outlive this.
    std::chrono::milliseconds poll_period{100}; ///< How often a change is looked for.
    std::chrono::milliseconds resend_period{1000}; ///< Longest gap between publications.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO}; ///< Log verbosity.
  };

  /// @brief Register the services and the status publisher, and start publishing.
  /// @param config The Arbiter, the participant, and the publication periods.
  explicit ArbitrationServer(const Config &config);

  /// @brief Whether every endpoint registered.
  /// @return True if the three services and the status publisher are all valid.
  bool is_valid() const { return valid_; }

  /// @brief Publish the status now if it changed, without waiting for the next poll.
  /// @note Call after acting on the Arbiter directly (a local grant or e-stop).
  void publish_status_if_changed();

private:
  using AcquireServer =
      espp::ServiceServer<rammp::AcquireControlRequest, rammp::AcquireControlResponse>;
  using ReleaseServer =
      espp::ServiceServer<rammp::ReleaseControlRequest, rammp::ReleaseControlResponse>;
  using RevokeServer =
      espp::ServiceServer<rammp::RevokeControlRequest, rammp::RevokeControlResponse>;

  rammp::AcquireControlResponse on_acquire(const rammp::AcquireControlRequest &request);
  rammp::ReleaseControlResponse on_release(const rammp::ReleaseControlRequest &request);
  rammp::RevokeControlResponse on_revoke(const rammp::RevokeControlRequest &request);
  void publish_status(bool force);

  Arbiter &arbiter_;
  const std::chrono::milliseconds resend_period_;
  std::mutex status_mutex_; ///< Serializes the publications and guards the two below.
  ArbitrationStatus last_status_{};
  std::chrono::steady_clock::time_point last_publish_{};
  bool valid_{false};
  std::unique_ptr<espp::Publisher<rammp::ControlStatus>> status_publisher_;
  std::unique_ptr<AcquireServer> acquire_server_;
  std::unique_ptr<ReleaseServer> release_server_;
  std::unique_ptr<RevokeServer> revoke_server_;
  std::unique_ptr<espp::Timer> status_timer_;
};

} // namespace mib
