#pragma once

#include <cstdint>

/// @file mib_panel_protocol.hpp
/// @brief The MIB control module on the service link (mib::ServiceLink), module 0x40.
///
/// The control panel (tools/mib_panel/index.html) mirrors these numbers; change both.
/// Frames are espp stream_frame, payloads little-endian, floats IEEE-754 f32.
///
/// host -> MIB
///   0x01 MOTOR_OVERRIDE [axis u8][requested_state u8][mode u8][0 u8]
///                       [position f32][velocity f32][torque f32]
///                       [torque_limit f32][vel_limit f32][accel_limit f32]
///        The MIB sends this MotorCommand to `axis` instead of its own, until 300 ms
///        after the last one: resend at >= 10 Hz. Accepted only in IDLE with the e-stop
///        clear; limits of 0 become the service ceilings in MIBconfig.hpp. No reply
///        unless refused (ERROR).
///   0x02 MOTOR_RELEASE  [axis u8]                 0xFF = every axis. Reply OK.
///   0x03 MOTOR_TUNING   [axis u8][persist u8][speed_kp f32][speed_ki f32][hall_pll_hz f32]
///        Published as MotorTuning with a new tuning_req_id, resent until the board
///        echoes it. Units and 0 / negative meanings are MotorTuning's. Reply OK
///        [type][req_id].
///   0x04 CLEAR_FAULT    [axis u8]                 a new clear_fault_req_id. Reply OK.
///   0x05 DRIVE          [request u8][profile u8]  rammp::DriveCommand. Reply OK / ERROR.
///   0x06 STICK          [x f32][y f32]            drive input in place of the HMI's stick,
///        same meaning as XYTwist x / y, until 300 ms after the last. In AUTONOMOUS a
///        touch ends autonomy, as the HMI's stick does. No reply unless refused.
///   0x07 SEAT           [axis u8][target f32]     rammp::SeatCommand. Reply OK / ERROR.
///   0x08 ESTOP          [action u8]               1 = engage, 2 = start a clear attempt.
///   0x09 STATUS                                    Reply STATUS_REPLY, UTF-8 JSON.
///   0x0A PARAMS                                    Reply PARAMS_REPLY: the MIB's tunable
///        params (today the push-assist gains, mib::push_assist::kParams), UTF-8 JSON
///        {"saved":bool,"params":[{"key","label","unit","group","type":"float","value",
///        "default","min","max","step"}, ...]}. The panel renders whatever is listed.
///   0x0B SET_PARAM      [key_len u8][key UTF-8][value f32]
///        Clamped to the param's range and live at once, in RAM. Reply PARAM_REPLY: that
///        param's JSON object after the set. ERROR for an unknown key.
///   0x0C SAVE_PARAMS                               Write the params to NVS. Reply
///        SAVE_REPLY [ok u8].
///   0x0D DEFAULT_PARAMS                            Back to the defaults in RAM (unsaved).
///        Reply PARAMS_REPLY.
///   The push-assist board serves the same params protocol on its module 0x30 (types 0x03
///   to 0x06; rammp-org/space-bar-push-assist firmware/INTERFACE.md).
///
/// MIB -> host
///   0x81 OK           [request type u8][value u8]
///   0x89 STATUS_REPLY UTF-8 JSON (see MibSystem::status_json)
///   0x8A PARAMS_REPLY UTF-8 JSON (see PARAMS)
///   0x8B PARAM_REPLY  UTF-8 JSON, one param
///   0x8C SAVE_REPLY   [ok u8]
///   0x8F ERROR        [request type u8][UTF-8 reason]
namespace mib::panel {

inline constexpr uint8_t kModule = 0x40;

inline constexpr uint8_t kMotorOverride = 0x01;
inline constexpr uint8_t kMotorRelease = 0x02;
inline constexpr uint8_t kMotorTuning = 0x03;
inline constexpr uint8_t kClearFault = 0x04;
inline constexpr uint8_t kDrive = 0x05;
inline constexpr uint8_t kStick = 0x06;
inline constexpr uint8_t kSeat = 0x07;
inline constexpr uint8_t kEstop = 0x08;
inline constexpr uint8_t kStatus = 0x09;
inline constexpr uint8_t kParams = 0x0A;
inline constexpr uint8_t kSetParam = 0x0B;
inline constexpr uint8_t kSaveParams = 0x0C;
inline constexpr uint8_t kDefaultParams = 0x0D;

inline constexpr uint8_t kOk = 0x81;
inline constexpr uint8_t kStatusReply = 0x89;
inline constexpr uint8_t kParamsReply = 0x8A;
inline constexpr uint8_t kParamReply = 0x8B;
inline constexpr uint8_t kSaveReply = 0x8C;
inline constexpr uint8_t kError = 0x8F;

inline constexpr uint8_t kAllAxes = 0xFF;
inline constexpr uint8_t kEstopEngage = 1;
inline constexpr uint8_t kEstopClear = 2;

} // namespace mib::panel
