// The MIB side of the control panel: the service link, the control module
// (mib_panel_protocol.hpp), the status JSON and the telemetry stream.

#include "mib_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "driver/temperature_sensor.h"
#include "esp_app_desc.h"

#include "MIBconfig.hpp"
#include "mib_panel_protocol.hpp"

using namespace std::chrono_literals;

namespace {

temperature_sensor_handle_t s_chip_temp = nullptr;

float get_f32(const std::vector<uint8_t> &p, size_t offset) {
  float v;
  std::memcpy(&v, p.data() + offset, sizeof(v));
  return v;
}

std::vector<uint8_t> reply_frame(uint8_t type, std::span<const uint8_t> payload) {
  return espp::stream_frame::build_frame(true, mib::panel::kModule, type, payload);
}

void reply_ok(const mib::ServiceLink::reply_fn &reply, uint8_t request, uint8_t value = 0) {
  const uint8_t p[2] = {request, value};
  reply(reply_frame(mib::panel::kOk, p));
}

void reply_error(const mib::ServiceLink::reply_fn &reply, uint8_t request, std::string_view why) {
  std::vector<uint8_t> p{request};
  p.insert(p.end(), why.begin(), why.end());
  reply(reply_frame(mib::panel::kError, p));
}

const char *board_state_name(rammp::BoardState s) {
  switch (s) {
  case rammp::BoardState::DISARMED:
    return "DISARMED";
  case rammp::BoardState::ARMED:
    return "ARMED";
  case rammp::BoardState::SAFE_STOPPING:
    return "SAFE_STOPPING";
  case rammp::BoardState::HOLDING:
    return "HOLDING";
  case rammp::BoardState::FAULT:
    return "FAULT";
  }
  return "UNKNOWN";
}

const char *fault_name(rammp::FaultCode f) {
  switch (f) {
  case rammp::FaultCode::NONE:
    return "NONE";
  case rammp::FaultCode::WATCHDOG:
    return "WATCHDOG";
  case rammp::FaultCode::OVERCURRENT:
    return "OVERCURRENT";
  case rammp::FaultCode::VDS_OCP:
    return "VDS_OCP";
  case rammp::FaultCode::OVERTEMP:
    return "OVERTEMP";
  case rammp::FaultCode::ENCODER:
    return "ENCODER";
  case rammp::FaultCode::DRV_FAULT:
    return "DRV_FAULT";
  case rammp::FaultCode::UNASSIGNED_AXIS:
    return "UNASSIGNED_AXIS";
  case rammp::FaultCode::SAMPLER:
    return "SAMPLER";
  case rammp::FaultCode::HALL:
    return "HALL";
  case rammp::FaultCode::STALL:
    return "STALL";
  }
  return "UNKNOWN";
}

// JSON has no NaN or infinity: report a non-finite reading as 0.
float fin(float v) { return std::isfinite(v) ? v : 0.0f; }

// JSON-safe copy of a short C string (the e-stop cause).
std::string json_escape(std::string_view s) {
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\')
      out.push_back('\\');
    if (static_cast<unsigned char>(c) >= 0x20)
      out.push_back(c);
  }
  return out;
}

// Telemetry channels per axis, in emit order: "<segment>.<name>".
constexpr const char *kAxisChannels[] = {"cmd_vel", "cmd_torque", "vel",  "pos",   "iq",
                                         "torque",  "vbus",       "temp", "state", "online"};
constexpr size_t kAxisChannelCount = std::size(kAxisChannels);
// MIB-wide channels, then the seat targets, then the axes.
constexpr const char *kMibChannels[] = {"mib.state",       "mib.estop",     "mib.profile",
                                        "mib.stick_x",     "mib.stick_y",   "sensor.mib_chip_temp",
                                        "seat.fb_tilt",    "seat.lat_tilt", "seat.elevation",
                                        "seat.translation"};

} // namespace

void MibSystem::start_service_link() {
  std::vector<std::string> channels(std::begin(kMibChannels), std::end(kMibChannels));
  for (const auto &spec : rammp::kAxes)
    for (const auto *name : kAxisChannels)
      channels.push_back(std::string(spec.segment) + "." + name);

  temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
  if (temperature_sensor_install(&tcfg, &s_chip_temp) != ESP_OK ||
      temperature_sensor_enable(s_chip_temp) != ESP_OK) {
    logger_.warn("Chip temperature sensor unavailable");
    s_chip_temp = nullptr;
  }

  service_ = std::make_unique<mib::ServiceLink>(mib::ServiceLink::Config{
      .product = "RAMMP MIB",
      .firmware = esp_app_get_description()->version,
      .channels = channels,
      .usb = mib::config::service_usb_enabled,
      .uart = mib::config::service_uart_enabled,
      .landing_page_url = mib::config::service_landing_page_url,
      .uart_telemetry_divider = mib::config::service_uart_telemetry_divider,
      .tcp_port = mib::config::service_tcp_port,
      .before_ota = [this]() { make_safe_for_ota(); },
      .ota_aborted =
          [this]() {
            ota_active_.store(false);
            logger_.info("Firmware update aborted: motors may run again");
          },
  });
  service_->register_module(
      mib::panel::kModule,
      [this](const espp::stream_frame::Frame &f, const mib::ServiceLink::reply_fn &reply) {
        handle_panel_frame(f, reply);
      },
      "MIB control", "Drive, seat, e-stop, motor overrides and tuning");
  service_->set_info_extra(
      [this]() { return fmt::format("\"ip\":\"{}\"", mib_.ethernet_ip_string()); });
  if (!service_->start()) {
    logger_.warn("Control panel link is down; the MIB runs without it");
    return;
  }

  telemetry_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex, std::condition_variable &cv) -> bool {
        if (service_->streaming())
          emit_telemetry();
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait_for(lock, mib::config::service_telemetry_interval);
        return false;
      },
      .task_config = {.name = "MIB telemetry", .stack_size_bytes = 4 * 1024, .priority = 1},
  });
  telemetry_task_->start();
}

void MibSystem::make_safe_for_ota() {
  // Flash writes stall the cache for milliseconds at a time: nothing should be moving.
  ota_active_.store(true);
  {
    std::lock_guard<std::mutex> lock(axes_mutex_);
    for (auto &axis : axes_)
      axis.override_until = {};
  }
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    panel_stick_until_ = {};
  }
  drive_controller_.stop();
  const auto state = state_.load();
  if (state == SystemState::DRIVE_ENABLED) {
    state_.store(SystemState::IDLE);
  } else if (state == SystemState::AUTONOMOUS || state == SystemState::SHARED) {
    leave_autonomy("firmware update");
  }
  logger_.warn("Firmware update: drive disabled, overrides dropped, motors commanded DISARMED");
}

bool MibSystem::any_override_active() const {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(axes_mutex_);
  return std::any_of(axes_.begin(), axes_.end(),
                     [now](const AxisLink &a) { return now < a.override_until; });
}

void MibSystem::handle_panel_frame(const espp::stream_frame::Frame &f,
                                   const mib::ServiceLink::reply_fn &reply) {
  namespace P = mib::panel;
  if (f.is_reply())
    return;
  const auto &p = f.payload;
  const auto need = [&](size_t n) {
    if (p.size() >= n)
      return true;
    reply_error(reply, f.type, "payload too short");
    return false;
  };
  const auto axis_ok = [&](uint8_t axis) {
    if (axis < rammp::kAxisCount)
      return true;
    reply_error(reply, f.type, "no such axis");
    return false;
  };

  switch (f.type) {
  case P::kMotorOverride: {
    if (!need(28) || !axis_ok(p[0]))
      return;
    if (state_.load() != SystemState::IDLE || latch_.is_active() || ota_active_.load()) {
      reply_error(reply, f.type,
                  latch_.is_active()   ? "e-stop is latched"
                  : ota_active_.load() ? "a firmware update is running"
                                       : "MIB is not IDLE");
      return;
    }
    if (!endpoints_ready_.load()) {
      reply_error(reply, f.type, "RTPS is not up");
      return;
    }
    rammp::MotorCommand c{};
    c.requested_state = static_cast<rammp::RequestedState>(p[1]);
    c.mode = static_cast<rammp::ControlMode>(p[2]);
    c.position = get_f32(p, 4);
    c.velocity = get_f32(p, 8);
    c.torque = get_f32(p, 12);
    // A limit of 0 is the board's own ceiling: the panel never gets more than the
    // service ceiling, whatever it asks for.
    const auto cap = [](float v, float ceiling) {
      return (v <= 0.0f || !std::isfinite(v)) ? ceiling : std::min(v, ceiling);
    };
    c.torque_limit = cap(get_f32(p, 16), mib::config::service_max_torque_nm);
    c.vel_limit = cap(get_f32(p, 20), mib::config::service_max_velocity_radps);
    c.accel_limit = std::max(0.0f, get_f32(p, 24));
    c.velocity = std::clamp(c.velocity, -c.vel_limit, c.vel_limit);
    c.torque = std::clamp(c.torque, -c.torque_limit, c.torque_limit);
    std::lock_guard<std::mutex> lock(axes_mutex_);
    axes_[p[0]].override_command = c;
    axes_[p[0]].override_until =
        std::chrono::steady_clock::now() + mib::config::service_override_timeout;
    return;
  }
  case P::kMotorRelease: {
    if (!need(1))
      return;
    std::lock_guard<std::mutex> lock(axes_mutex_);
    for (size_t i = 0; i < rammp::kAxisCount; i++)
      if (p[0] == P::kAllAxes || p[0] == i)
        axes_[i].override_until = {};
    reply_ok(reply, f.type);
    return;
  }
  case P::kMotorTuning: {
    if (!need(14) || !axis_ok(p[0]))
      return;
    std::lock_guard<std::mutex> lock(axes_mutex_);
    if (++tuning_req_ == 0) // 0 means "none yet" in tuning_ack
      tuning_req_ = 1;
    auto &axis = axes_[p[0]];
    axis.tuning = rammp::MotorTuning{.tuning_req_id = tuning_req_,
                                     .persist = static_cast<uint8_t>(p[1] ? 1 : 0),
                                     .reserved0 = 0,
                                     .reserved1 = 0,
                                     .speed_kp = get_f32(p, 2),
                                     .speed_ki = get_f32(p, 6),
                                     .hall_pll_hz = get_f32(p, 10)};
    axis.tuning_until = std::chrono::steady_clock::now() + mib::config::tuning_resend_timeout;
    logger_.info("Panel tuning {}: kp={} ki={} pll={} persist={} (req {})",
                 rammp::kAxes[p[0]].segment, axis.tuning.speed_kp, axis.tuning.speed_ki,
                 axis.tuning.hall_pll_hz, axis.tuning.persist, tuning_req_);
    reply_ok(reply, f.type, tuning_req_);
    return;
  }
  case P::kClearFault: {
    if (!need(1) || !axis_ok(p[0]))
      return;
    std::lock_guard<std::mutex> lock(axes_mutex_);
    if (++axes_[p[0]].clear_fault_req == 0)
      axes_[p[0]].clear_fault_req = 1;
    reply_ok(reply, f.type, axes_[p[0]].clear_fault_req);
    return;
  }
  case P::kDrive: {
    if (!need(2))
      return;
    const rammp::DriveCommand command{.request = static_cast<rammp::DriveRequest>(p[0]),
                                      .profile = static_cast<rammp::DriveProfile>(p[1]),
                                      .push_assist = false}; // the panel has no push-assist control
    if (handle_drive_command(command))
      reply_ok(reply, f.type);
    else
      reply_error(reply, f.type, "refused: see the log");
    return;
  }
  case P::kStick: {
    if (!need(8))
      return;
    const float x = std::clamp(get_f32(p, 0), -1.0f, 1.0f);
    const float y = std::clamp(get_f32(p, 4), -1.0f, 1.0f);
    const rammp::XYTwist sample{.x = x, .y = y, .twist = 0.0f, .buttons = rammp::Buttons::NONE};
    if (state_.load() == SystemState::AUTONOMOUS) {
      // As the HMI joystick: a touch ends autonomy (back to IDLE), and does not drive.
      apply_stick(sample);
      return;
    }
    if (state_.load() != SystemState::DRIVE_ENABLED) {
      reply_error(reply, f.type, "drive is not enabled");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(joystick_mutex_);
      panel_stick_x_ = x;
      panel_stick_y_ = y;
      panel_stick_until_ = std::chrono::steady_clock::now() + mib::config::service_override_timeout;
    }
    apply_stick(sample);
    return;
  }
  case P::kSeat: {
    if (!need(5))
      return;
    const rammp::SeatCommand command{.axis = static_cast<rammp::SeatAxis>(p[0]),
                                     .target = get_f32(p, 1)};
    if (handle_seat_control_command(command))
      reply_ok(reply, f.type);
    else
      reply_error(reply, f.type, "refused: seat moves only in IDLE");
    return;
  }
  case P::kEstop: {
    if (!need(1))
      return;
    if (p[0] == P::kEstopEngage) {
      latch_.engage("panel");
      logger_.warn("E-stop engaged from the control panel");
    } else if (p[0] == P::kEstopClear) {
      latch_.start_clear_attempt();
      logger_.warn("E-stop clear attempt from the control panel");
    } else {
      reply_error(reply, f.type, "unknown action");
      return;
    }
    reply_ok(reply, f.type, p[0]);
    return;
  }
  case P::kStatus: {
    const auto json = status_json();
    reply(reply_frame(
        P::kStatusReply,
        std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(json.data()), json.size())));
    return;
  }
  default:
    reply_error(reply, f.type, "unknown request");
    return;
  }
}

std::string MibSystem::status_json() {
  const char *state = "UNKNOWN";
  switch (state_.load()) {
  case SystemState::INIT:
    state = "INIT";
    break;
  case SystemState::IDLE:
    state = "IDLE";
    break;
  case SystemState::CALIBRATE:
    state = "CALIBRATE";
    break;
  case SystemState::DRIVE_ENABLED:
    state = "DRIVE_ENABLED";
    break;
  case SystemState::AUTONOMOUS:
    state = "AUTONOMOUS";
    break;
  case SystemState::SHARED:
    state = "SHARED";
    break;
  case SystemState::ESTOP:
    state = "ESTOP";
    break;
  case SystemState::ERROR:
    state = "ERROR";
    break;
  }
  const auto latch = latch_.status();
  const auto now = std::chrono::steady_clock::now();
  bool stick_live = false;
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    stick_live = now < panel_stick_until_;
  }
  rammp::SeatState seat{};
  {
    std::lock_guard<std::mutex> lock(seat_state_mutex_);
    seat = seat_state_;
  }

  std::string json =
      fmt::format("{{\"state\":\"{}\",\"profile\":{},\"ethernet\":{},\"ip\":\"{}\",\"rtps\":{},"
                  "\"estop\":{{\"latched\":{},\"clearing\":{},\"cause\":\"{}\",\"releases\":{}}},"
                  "\"panel_stick\":{},\"seat\":[{:.2f},{:.2f},{:.2f},{:.2f}],\"axes\":[",
                  state, static_cast<int>(drive_profile_.load()),
                  mib_.ethernet_connected() ? "true" : "false", mib_.ethernet_ip_string(),
                  endpoints_ready_.load() ? "true" : "false", latch.latched ? "true" : "false",
                  latch.clear_attempt_running ? "true" : "false", json_escape(latch.cause),
                  latch.releases, stick_live ? "true" : "false", fin(seat.front_back_tilt),
                  fin(seat.lateral_tilt), fin(seat.elevation), fin(seat.translation));

  // Each board's last MotorState (copied out of handle_motor_state()'s cache first, so the
  // two mutexes are never held together).
  std::array<rammp::MotorState, rammp::kAxisCount> states{};
  std::array<std::chrono::steady_clock::time_point, rammp::kAxisCount> state_times{};
  {
    std::lock_guard<std::mutex> lock(motor_state_mutex_);
    states = motor_states_;
    state_times = motor_state_times_;
  }
  std::lock_guard<std::mutex> lock(axes_mutex_);
  for (size_t i = 0; i < rammp::kAxisCount; i++) {
    const auto &a = axes_[i];
    const auto &s = states[i];
    const bool seen = state_times[i] != std::chrono::steady_clock::time_point{};
    const auto age_ms =
        seen ? std::chrono::duration_cast<std::chrono::milliseconds>(now - state_times[i]).count()
             : -1;
    const bool online = seen && age_ms < mib::config::motor_state_timeout.count();
    const float tmax = *std::max_element(s.temps.begin(), s.temps.end());
    json += fmt::format(
        "{}{{\"axis\":{},\"segment\":\"{}\",\"label\":\"{}\",\"online\":{},\"age_ms\":{},"
        "\"api\":{},\"fw\":\"{}.{}.{}\",\"state\":\"{}\",\"mode\":{},\"fault\":\"{}\","
        "\"drv_status\":{},\"cmd_age_ms\":{},\"uptime_ms\":{},\"vbus\":{:.2f},\"temp_max\":{:.1f},"
        "\"limits\":[{:.3f},{:.3f},{:.3f}],\"tuning\":{{\"ack\":{},\"req\":{},\"kp\":{:.5f},"
        "\"ki\":{:.5f},\"pll\":{:.2f}}},\"override\":{},\"clear_req\":{},\"clear_ack\":{},"
        "\"cmd\":{{\"state\":{},\"mode\":{},\"vel\":{:.3f}}}}}",
        i ? "," : "", i, rammp::kAxes[i].segment, rammp::kAxes[i].label, online ? "true" : "false",
        age_ms, s.api_version, s.fw_version >> 24, (s.fw_version >> 16) & 0xFF,
        s.fw_version & 0xFFFF, board_state_name(s.state), static_cast<int>(s.mode),
        fault_name(s.fault_code), s.drv_status, s.cmd_age_ms, s.uptime_ms, fin(s.vbus), fin(tmax),
        fin(s.torque_limit_eff), fin(s.vel_limit_eff), fin(s.accel_limit_eff), s.tuning_ack,
        a.tuning.tuning_req_id, fin(s.speed_kp), fin(s.speed_ki), fin(s.hall_pll_hz),
        now < a.override_until ? "true" : "false", a.clear_fault_req, s.clear_fault_ack,
        static_cast<int>(a.last_command.requested_state), static_cast<int>(a.last_command.mode),
        fin(a.last_command.velocity));
  }
  json += "]}";
  return json;
}

void MibSystem::emit_telemetry() {
  std::vector<float> v;
  v.reserve(std::size(kMibChannels) + rammp::kAxisCount * kAxisChannelCount);
  v.push_back(static_cast<float>(state_.load()));
  v.push_back(latch_.is_active() ? 1.0f : 0.0f);
  v.push_back(static_cast<float>(drive_profile_.load()));
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    const bool panel = std::chrono::steady_clock::now() < panel_stick_until_;
    v.push_back(panel ? panel_stick_x_ : latest_joystick_.x);
    v.push_back(panel ? panel_stick_y_ : latest_joystick_.y);
  }
  float chip_c = NAN;
  if (s_chip_temp)
    temperature_sensor_get_celsius(s_chip_temp, &chip_c);
  v.push_back(chip_c);
  {
    std::lock_guard<std::mutex> lock(seat_state_mutex_);
    v.push_back(seat_state_.front_back_tilt);
    v.push_back(seat_state_.lateral_tilt);
    v.push_back(seat_state_.elevation);
    v.push_back(seat_state_.translation);
  }
  const auto now = std::chrono::steady_clock::now();
  // Each board's last MotorState (copied out of handle_motor_state()'s cache first, so the
  // two mutexes are never held together).
  std::array<rammp::MotorState, rammp::kAxisCount> states{};
  std::array<std::chrono::steady_clock::time_point, rammp::kAxisCount> state_times{};
  {
    std::lock_guard<std::mutex> lock(motor_state_mutex_);
    states = motor_states_;
    state_times = motor_state_times_;
  }
  std::lock_guard<std::mutex> lock(axes_mutex_);
  for (size_t i = 0; i < rammp::kAxisCount; i++) {
    const auto &a = axes_[i];
    const auto &s = states[i];
    const bool online = state_times[i] != std::chrono::steady_clock::time_point{} &&
                        now - state_times[i] < mib::config::motor_state_timeout;
    v.push_back(a.last_command.velocity);
    v.push_back(a.last_command.torque);
    v.push_back(online ? s.velocity : NAN);
    v.push_back(online ? s.position : NAN);
    v.push_back(online ? s.iq : NAN);
    v.push_back(online ? s.torque_est : NAN);
    v.push_back(online ? s.vbus : NAN);
    v.push_back(online ? *std::max_element(s.temps.begin(), s.temps.end()) : NAN);
    v.push_back(online ? static_cast<float>(s.state) : NAN);
    v.push_back(online ? 1.0f : 0.0f);
  }
  service_->emit(v);
}
