#include "arbitration_server.hpp"

#include <ctime>
#include <optional>

namespace mib {

namespace {

// The stamp is ignored: only a change in who may command the base is news.
bool same(const ArbitrationStatus &a, const ArbitrationStatus &b) {
  return a.mode == b.mode && a.estopped == b.estopped && a.owned == b.owned &&
         a.owner_id == b.owner_id && a.generation == b.generation &&
         a.rejected_count == b.rejected_count;
}

// Wall time if the MIB has been told it, else all-zero ("unstamped"). Before that the
// clock counts from 1970 at boot, which a reader would take for a real time.
rammp::Time now() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  constexpr time_t kClockSet = 1'700'000'000; // 2023-11-14
  if (ts.tv_sec < kClockSet) {
    return {0, 0};
  }
  return {static_cast<int32_t>(ts.tv_sec), static_cast<uint32_t>(ts.tv_nsec)};
}

} // namespace

ArbitrationServer::ArbitrationServer(const Config &config)
    : BaseComponent("ArbitrationServer", config.log_level)
    , arbiter_(config.arbiter)
    , resend_period_(config.resend_period) {
  status_publisher_ = std::make_unique<espp::Publisher<rammp::ControlStatus>>(
      config.participant, espp::Publisher<rammp::ControlStatus>::Config{
                              .topic = rammp::kBaseControlStatus.name,
                              .type_name = rammp::kBaseControlStatus.type,
                              .reliability = espp::RtpsParticipant::Reliability::RELIABLE});

  acquire_server_ = std::make_unique<AcquireServer>(
      config.participant,
      AcquireServer::Config{.service = rammp::kBaseAcquireControl.name,
                            .type_name = rammp::kBaseAcquireControl.type,
                            .handler = [this](const rammp::AcquireControlRequest &request) {
                              return on_acquire(request);
                            }});
  release_server_ = std::make_unique<ReleaseServer>(
      config.participant,
      ReleaseServer::Config{.service = rammp::kBaseReleaseControl.name,
                            .type_name = rammp::kBaseReleaseControl.type,
                            .handler = [this](const rammp::ReleaseControlRequest &request) {
                              return on_release(request);
                            }});
  revoke_server_ = std::make_unique<RevokeServer>(
      config.participant,
      RevokeServer::Config{.service = rammp::kBaseRevokeControl.name,
                           .type_name = rammp::kBaseRevokeControl.type,
                           .handler = [this](const rammp::RevokeControlRequest &request) {
                             return on_revoke(request);
                           }});

  valid_ = status_publisher_->is_valid() && acquire_server_->is_valid() &&
           release_server_->is_valid() && revoke_server_->is_valid();
  if (!valid_) {
    logger_.error("Failed to register the arbitration endpoints: status={} acquire={} "
                  "release={} revoke={}",
                  status_publisher_->is_valid(), acquire_server_->is_valid(),
                  release_server_->is_valid(), revoke_server_->is_valid());
    return;
  }

  publish_status(true);
  status_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "arbitration status",
      .period = config.poll_period,
      .callback =
          [this] {
            publish_status_if_changed();
            return false;
          },
      .stack_size_bytes = 4 * 1024,
  });
  logger_.info("Serving {}, {}, {} and {}", rammp::kBaseAcquireControl.name,
               rammp::kBaseReleaseControl.name, rammp::kBaseRevokeControl.name,
               rammp::kBaseControlStatus.name);
}

void ArbitrationServer::publish_status_if_changed() { publish_status(false); }

rammp::AcquireControlResponse
ArbitrationServer::on_acquire(const rammp::AcquireControlRequest &request) {
  // All-zero asks the Arbiter to mint; anything else is a key to adopt.
  const bool supplied = request.token != Token{};
  const auto grant = arbiter_.grant(request.owner_id,
                                    supplied ? std::optional<Token>(request.token) : std::nullopt);
  publish_status_if_changed();
  return {grant.accepted, grant.token, grant.generation, grant.message};
}

rammp::ReleaseControlResponse
ArbitrationServer::on_release(const rammp::ReleaseControlRequest &request) {
  const bool released = arbiter_.release(request.token);
  publish_status_if_changed();
  return {released, released ? "" : "token does not match the current owner"};
}

rammp::RevokeControlResponse
ArbitrationServer::on_revoke(const rammp::RevokeControlRequest &request) {
  logger_.warn("Operator revoke: {}",
               request.reason.empty() ? "(no reason given)" : request.reason);
  arbiter_.revoke();
  publish_status_if_changed();
  // Always true, as on the arm: afterwards nobody holds the base, whoever did before.
  return {true, ""};
}

void ArbitrationServer::publish_status(bool force) {
  const auto status = arbiter_.status();
  const auto now_steady = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (!force && same(status, last_status_) && now_steady - last_publish_ < resend_period_) {
    return;
  }
  rammp::ControlStatus message{
      .header = {.stamp = now(), .frame_id = ""},
      .arbitration_enabled = status.mode == ArbitrationMode::ENFORCED,
      .estopped = status.estopped,
      .owned = status.owned,
      .owner_id = status.owner_id,
      .generation = status.generation,
      .rejected_count = status.rejected_count,
  };
  if (!status_publisher_->publish(message)) {
    logger_.warn("Failed to publish the control status");
    return;
  }
  last_status_ = status;
  last_publish_ = now_steady;
}

} // namespace mib
