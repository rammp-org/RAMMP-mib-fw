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
///        same meaning as XYTwist x / y, until 300 ms after the last. No reply unless
///        refused.
///   0x07 SEAT           [axis u8][target f32]     rammp::SeatCommand. Reply OK / ERROR.
///   0x08 ESTOP          [action u8]               1 = engage, 2 = start a clear attempt.
///   0x09 STATUS                                    Reply STATUS_REPLY, UTF-8 JSON.
///
/// MIB -> host
///   0x81 OK           [request type u8][value u8]
///   0x89 STATUS_REPLY UTF-8 JSON (see MibSystem::status_json)
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

inline constexpr uint8_t kOk = 0x81;
inline constexpr uint8_t kStatusReply = 0x89;
inline constexpr uint8_t kError = 0x8F;

inline constexpr uint8_t kAllAxes = 0xFF;
inline constexpr uint8_t kEstopEngage = 1;
inline constexpr uint8_t kEstopClear = 2;

} // namespace mib::panel
