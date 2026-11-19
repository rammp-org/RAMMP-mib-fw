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
///   0x0A LEG_MOVE       [leg u8][relative u8][counts i32]  Move one leg actuator
///        (MIBconfig.hpp `Leg` order): relative 0 = to the absolute encoder count,
///        1 = by `counts` from where the leg is now (the encoder is read first). Clamped
///        to the leg's range by the actuator. The panel's "Go to Home" is one of these per
///        leg, from the MIB poses in config/poses.json. In IDLE with the e-stop clear
///        only. Reply OK / ERROR [reason].
///   0x0B LEG_RANGE      [leg u8][min i32][max i32]  Install one leg's calibrated range,
///        in encoder counts: the actuator clamps its targets to it, writes it to the
///        controller and saves it in the MIB's NVS, which overrides the compiled default in
///        MIBconfig.hpp from then on. STATUS reports it as the leg's min / max. In IDLE
///        only. Reply OK [leg] / ERROR [reason].
///   0x0C LEG_JOG        [leg u8][counts i32]      Jog one leg by `counts` from where it is
///        (the encoder is read first), or on from its previous target while that still
///        lies ahead, so a run of these adds up; 0 retargets it to where it is, which
///        ends a jog without leaving a stale target in the controller. A jog is the one move the
///        actuator accepts with its clamp lifted (LEG_CALIBRATE), so the panel's
///        Calibration tab sends these while a jog button is held, and a 0 on release. In
///        IDLE with the e-stop clear only. Reply OK [leg] / ERROR [reason].
///   0x0D LEG_CALIBRATE  [leg u8][on u8]           1 = lift one leg's range clamp on the
///        controller so jogs can reach its mechanical ends, 0 = restore it. Off at every
///        boot. STATUS reports it as the leg's `cal`. In IDLE only. Reply OK [leg] / ERROR.
///   0x0E SDO_READ       [node u8][index u16][sub u8][bytes u8]  Read one object from any
///        CANopen node (bytes 1, 2 or 4, sign-extended). Reads only: for bringing up a chair
///        whose controllers are not known yet. Any state but a firmware update. Reply
///        SDO_VALUE / ERROR [reason].
///   0x0F SCAN           [first u8][last u8]       Ask node ids first..last for 0x1000
///        (~60 ms per silent id: scan in chunks). Reply SCAN_REPLY.
///
/// MIB -> host
///   0x81 OK           [request type u8][value u8]
///   0x8D SCAN_REPLY   [count u8] then count x [node u8][device type u32]
///   0x8E SDO_VALUE    [node u8][index u16][sub u8][value i32]
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
inline constexpr uint8_t kLegMove = 0x0A;
inline constexpr uint8_t kLegRange = 0x0B;
inline constexpr uint8_t kLegJog = 0x0C;
inline constexpr uint8_t kLegCalibrate = 0x0D;
inline constexpr uint8_t kSdoRead = 0x0E;
inline constexpr uint8_t kScan = 0x0F;

inline constexpr uint8_t kOk = 0x81;
inline constexpr uint8_t kScanReply = 0x8D;
inline constexpr uint8_t kSdoValue = 0x8E;
inline constexpr uint8_t kStatusReply = 0x89;
inline constexpr uint8_t kError = 0x8F;

inline constexpr uint8_t kAllAxes = 0xFF;
inline constexpr uint8_t kEstopEngage = 1;
inline constexpr uint8_t kEstopClear = 2;

} // namespace mib::panel
