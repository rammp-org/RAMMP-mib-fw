/**
 * @file mib_params.hpp
 * @brief Runtime-tunable MIB parameters and the RTPS messages that read and write them.
 *
 * Every value in MIBconfig.hpp that a bench operator might want to change without a
 * reflash lives here as well, as a mutable copy seeded from the constexpr default. A
 * ParamSet message changes one value; the MIB validates it, applies it at once, and
 * reports the whole table back in the next ParamState so the sender can confirm.
 *
 * Values are not persisted: a reboot returns every parameter to its compiled default.
 * That is deliberate for a debug path. Promote a value into MIBconfig.hpp once it is
 * settled.
 *
 * Wire rules match the other RAMMP messages: each struct IS the wire layout, XCDR1,
 * fields in declaration order, every enum one byte wide. Every parameter is carried
 * as a float so the table can grow without touching the message; booleans are 0 or 1.
 *
 * To add a parameter, add ONE row to MIB_PARAM_TABLE. tools/mib_debugger mirrors this
 * table in mib_messages.py and must be updated in step.
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "MIBconfig.hpp"

/* Topics and types, in the style of the shared message headers. */
#define RAMMP_TOPIC_JOYSTICK_PARAM_SET "rammp/joystick/param_set"
#define RAMMP_TYPE_PARAM_SET "rammp/msg/ParamSet"
#define RAMMP_TOPIC_MIB_PARAMS "rammp/mib/params"
#define RAMMP_TYPE_PARAM_STATE "rammp/msg/ParamState"

namespace mib {

/* X(id, NAME, label, unit, min, max, default_expression)
   Booleans use min 0, max 1. The default expressions reference MIBconfig.hpp so the two
   files cannot drift apart. */
#define MIB_PARAM_TABLE(X)                                                                         \
  X(0,  INVERT_LEFT,           "Invert left wheel",     "bool",   0.0f,  1.0f,                    \
    config::differential_drive.invert_left ? 1.0f : 0.0f)                                          \
  X(1,  INVERT_RIGHT,          "Invert right wheel",    "bool",   0.0f,  1.0f,                    \
    config::differential_drive.invert_right ? 1.0f : 0.0f)                                         \
  X(2,  WHEEL_DIAMETER_M,      "Wheel diameter",        "m",      0.01f, 2.0f,                    \
    config::differential_drive.wheel_diameter_m)                                                   \
  X(3,  WHEEL_SEPARATION_M,    "Wheel separation",      "m",      0.01f, 3.0f,                    \
    config::differential_drive.wheel_separation_m)                                                 \
  X(4,  JOYSTICK_X_SIGN,       "Joystick X sign",       "+1/-1",  -1.0f, 1.0f, 1.0f)              \
  X(5,  LOW_MAX_LINEAR_V,      "LOW max linear",        "m/s",    0.0f,  5.0f,                    \
    config::drive_profile_low.max_linear_velocity_mps)                                             \
  X(6,  LOW_MAX_ANGULAR_V,     "LOW max angular",       "rad/s",  0.0f,  10.0f,                   \
    config::drive_profile_low.max_angular_velocity_radps)                                          \
  X(7,  LOW_MAX_LINEAR_A,      "LOW linear accel",      "m/s^2",  0.01f, 10.0f,                   \
    config::drive_profile_low.max_linear_acceleration_mps2)                                        \
  X(8,  LOW_MAX_ANGULAR_A,     "LOW angular accel",     "rad/s^2", 0.01f, 20.0f,                  \
    config::drive_profile_low.max_angular_acceleration_radps2)                                     \
  X(9,  NORMAL_MAX_LINEAR_V,   "NORMAL max linear",     "m/s",    0.0f,  5.0f,                    \
    config::drive_profile_normal.max_linear_velocity_mps)                                          \
  X(10, NORMAL_MAX_ANGULAR_V,  "NORMAL max angular",    "rad/s",  0.0f,  10.0f,                   \
    config::drive_profile_normal.max_angular_velocity_radps)                                       \
  X(11, NORMAL_MAX_LINEAR_A,   "NORMAL linear accel",   "m/s^2",  0.01f, 10.0f,                   \
    config::drive_profile_normal.max_linear_acceleration_mps2)                                     \
  X(12, NORMAL_MAX_ANGULAR_A,  "NORMAL angular accel",  "rad/s^2", 0.01f, 20.0f,                  \
    config::drive_profile_normal.max_angular_acceleration_radps2)                                  \
  X(13, HIGH_MAX_LINEAR_V,     "HIGH max linear",       "m/s",    0.0f,  5.0f,                    \
    config::drive_profile_high.max_linear_velocity_mps)                                            \
  X(14, HIGH_MAX_ANGULAR_V,    "HIGH max angular",      "rad/s",  0.0f,  10.0f,                   \
    config::drive_profile_high.max_angular_velocity_radps)                                         \
  X(15, HIGH_MAX_LINEAR_A,     "HIGH linear accel",     "m/s^2",  0.01f, 10.0f,                   \
    config::drive_profile_high.max_linear_acceleration_mps2)                                       \
  X(16, HIGH_MAX_ANGULAR_A,    "HIGH angular accel",    "rad/s^2", 0.01f, 20.0f,                  \
    config::drive_profile_high.max_angular_acceleration_radps2)

enum class ParamId : uint8_t {
#define MIB_PARAM_ID(id_, name_, ...) name_ = id_,
  MIB_PARAM_TABLE(MIB_PARAM_ID)
#undef MIB_PARAM_ID
};

struct ParamSpec {
  ParamId id;
  const char *name;  /**< "INVERT_LEFT" */
  const char *label; /**< "Invert left wheel" */
  const char *unit;  /**< "bool", "m", "rad/s" */
  float min_value;
  float max_value;
  float default_value;
};

inline constexpr std::array kParamSpecs{
#define MIB_PARAM_ROW(id_, name_, label_, unit_, min_, max_, default_)                            \
  ParamSpec{ParamId::name_, #name_, label_, unit_, min_, max_, default_},
    MIB_PARAM_TABLE(MIB_PARAM_ROW)
#undef MIB_PARAM_ROW
};
inline constexpr size_t kParamCount = kParamSpecs.size();

/* -------------------------------------------------------------------------
 * Messages
 * ---------------------------------------------------------------------- */

/** Set one parameter. joystick -> MIB, sent reliably, 8 bytes on the wire. */
struct ParamSet {
  uint8_t seq;  /**< +1 per request, wraps; echoed as ParamState::last_set_seq */
  ParamId id;   /**< which parameter */
  float value;  /**< the new value; booleans are 0 or 1 */
};

/** The whole table. MIB -> joystick, resent periodically, best-effort. */
struct ParamState {
  uint8_t seq;                            /**< +1 per message, wraps */
  uint8_t last_set_seq;                   /**< seq of the last ParamSet acted on; 0 = none yet */
  uint8_t last_set_ok;                    /**< 1 if that ParamSet was applied, 0 if rejected */
  uint8_t count;                          /**< kParamCount, so a reader can detect a table mismatch */
  std::array<float, kParamCount> values;  /**< indexed by ParamId */
};

/* -------------------------------------------------------------------------
 * Store: the live values, guarded by a mutex because RTPS callbacks, the state task
 * and the publication task all touch it.
 * ---------------------------------------------------------------------- */

class Params {
public:
  static Params &instance() {
    static Params params;
    return params;
  }

  float get(ParamId id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return values_[static_cast<size_t>(id)];
  }

  bool get_bool(ParamId id) const { return get(id) >= 0.5f; }

  /** Validate and store. Returns false, changing nothing, for an unknown id, a NaN,
      or a value outside the row's range. */
  bool set(ParamId id, float value) {
    const auto index = static_cast<size_t>(id);
    if (index >= kParamCount || std::isnan(value)) {
      return false;
    }
    const auto &spec = kParamSpecs[index];
    if (value < spec.min_value || value > spec.max_value) {
      return false;
    }
    if (spec.unit[0] == 'b') { // "bool": store exactly 0 or 1
      value = value >= 0.5f ? 1.0f : 0.0f;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    values_[index] = value;
    return true;
  }

  std::array<float, kParamCount> snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return values_;
  }

  void reset_to_defaults() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < kParamCount; ++i) {
      values_[i] = kParamSpecs[i].default_value;
    }
  }

private:
  Params() { reset_to_defaults(); }
  mutable std::mutex mutex_;
  std::array<float, kParamCount> values_{};
};

} // namespace mib
