"""A stand-in for the MIB firmware, for testing the debugger without hardware.

This mirrors the behaviour of MibSystem in components/MibSystem: the same
topics, the same state machine and the same guards, so the debugger sees a
board that behaves like the real one. It is a development aid, not a
simulation of the chassis. The trajectory planner is approximated by a simple
acceleration limit, and no seat motion is modelled.

Run it in one terminal and the debugger in another. Both default to domain 0
and discover each other over loopback multicast.
"""

import argparse
import math
import signal
import sys
import time

import mib_messages as msg

# Geometry and profile limits copied from components/MIBConfig/include/MIBconfig.hpp.
WHEEL_DIAMETER_M = 0.254
WHEEL_SEPARATION_M = 0.558
INVERT_LEFT = True
INVERT_RIGHT = False

PROFILE_LIMITS = {
    int(msg.DriveProfile.LOW): (0.35, 1.0, 0.5, 1.0),
    int(msg.DriveProfile.NORMAL): (0.75, 2.0, 1.0, 2.0),
    int(msg.DriveProfile.HIGH): (1.0, 3.14159, 2.0, 4.0),
}

PUBLICATION_HZ = 2.0    # MibStatus rate, matching publication_task_interval
MOTOR_COMMAND_HZ = 20.0  # matching motor_command_task_interval
STATE_HZ = 50.0          # matching state_task_interval

RPM_TO_RAD_PER_S = 2.0 * math.pi / 60.0


def wheel_speeds(linear_mps: float, angular_radps: float, params):
    """Differential drive, mirroring components/DifferentialDrive, using live params."""
    invert_left, invert_right, diameter, separation = params[0] >= 0.5, params[1] >= 0.5, params[2], params[3]
    half = separation / 2.0
    left_mps = linear_mps - angular_radps * half
    right_mps = linear_mps + angular_radps * half
    circumference = math.pi * diameter
    left_rpm = (left_mps / circumference) * 60.0
    right_rpm = (right_mps / circumference) * 60.0
    if invert_left:
        left_rpm = -left_rpm
    if invert_right:
        right_rpm = -right_rpm
    return left_rpm, right_rpm


class FakeMib:
    def __init__(self, domain_id=0, interface=None, boot_delay=1.0, local=False):
        self.domain_id = domain_id
        self.interface = interface
        self.local = local
        self.boot_delay = boot_delay

        self.state = msg.MibSystemState.INITIALIZING
        self.profile = int(msg.DriveProfile.NORMAL)
        self.seat = msg.SeatState()
        self.seq = 0
        self.motor_seq = 0

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Runtime parameters, mirroring mib::Params in the firmware.
        self.params = list(msg.PARAM_DEFAULTS)
        self.param_seq = 0
        self.last_set_seq = 0
        self.last_set_ok = 0

        self.running = True

    def start(self):
        from cyclonedds.core import Policy, Qos
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter, Publisher
        from cyclonedds.sub import DataReader, Subscriber
        from cyclonedds.topic import Topic

        import mib_link
        uri = mib_link.build_cyclonedds_uri(self.interface, self.local)
        if uri:
            import os
            os.environ["CYCLONEDDS_URI"] = uri

        from cyclonedds.util import duration
        qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(8))
        reliable = Qos(Policy.Reliability.Reliable(duration(seconds=1)), Policy.History.KeepLast(8))
        participant = DomainParticipant(self.domain_id)
        pub = Publisher(participant)
        sub = Subscriber(participant)

        def writer(name, dt, q=qos):
            return DataWriter(pub, Topic(participant, name, dt, qos=q), qos=q)

        def reader(name, dt, q=qos):
            return DataReader(sub, Topic(participant, name, dt, qos=q), qos=q)

        self.status_writer = writer(msg.TOPIC_MIB_STATUS, msg.MibStatus)
        self.motor_writers = {
            axis: writer(msg.motor_command_topic(axis), msg.MotorCommand)
            for axis in (2, 3)
        }
        self.joystick_reader = reader(msg.TOPIC_JOYSTICK_XY_TWIST, msg.XYTwist)
        self.drive_reader = reader(msg.TOPIC_JOYSTICK_DRIVE_COMMAND, msg.DriveCommand)
        self.seat_reader = reader(msg.TOPIC_JOYSTICK_SEAT_COMMAND, msg.SeatCommand)
        self.param_set_reader = reader(msg.TOPIC_JOYSTICK_PARAM_SET, msg.ParamSet, reliable)
        self.param_state_writer = writer(msg.TOPIC_MIB_PARAMS, msg.ParamState)

        print(f"fake MIB up on domain {self.domain_id}; INIT for {self.boot_delay:.1f}s")

    def run(self):
        boot_done = time.time() + self.boot_delay
        next_status = time.time()
        next_motor = time.time()
        last_step = time.time()

        while self.running:
            now = time.time()

            if self.state == msg.MibSystemState.INITIALIZING and now >= boot_done:
                self.state = msg.MibSystemState.IDLE
                print("state -> IDLE")

            self._drain_inputs()

            dt = now - last_step
            last_step = now
            self._advance_motion(dt)

            if now >= next_status:
                next_status = now + 1.0 / PUBLICATION_HZ
                self._publish_status()
                self._publish_params()

            if now >= next_motor:
                next_motor = now + 1.0 / MOTOR_COMMAND_HZ
                self._publish_motor_commands()

            time.sleep(1.0 / STATE_HZ)

    def _drain_inputs(self):
        from mib_link import valid_samples
        for sample in valid_samples(self.drive_reader, msg.DriveCommand):
            self._handle_drive(sample)
        for sample in valid_samples(self.seat_reader, msg.SeatCommand):
            self._handle_seat(sample)
        for sample in valid_samples(self.param_set_reader, msg.ParamSet):
            self._handle_param_set(sample)
        samples = valid_samples(self.joystick_reader, msg.XYTwist, count=20)
        if samples:
            self._handle_joystick(samples[-1])

    def _handle_drive(self, command):
        if command.request == int(msg.DriveRequest.ENABLE):
            if self.state not in (msg.MibSystemState.IDLE, msg.MibSystemState.ENABLED):
                print("ignoring ENABLE outside IDLE")
                return
            self.profile = command.profile
            if self.state == msg.MibSystemState.IDLE:
                self.state = msg.MibSystemState.ENABLED
                print(f"state -> ENABLED, profile {msg.DriveProfile(self.profile).name}")
            else:
                print(f"profile -> {msg.DriveProfile(self.profile).name}")
        else:
            if self.state != msg.MibSystemState.ENABLED:
                print("ignoring DISABLE outside ENABLED")
                return
            self.target_linear = self.target_angular = 0.0
            self.state = msg.MibSystemState.IDLE
            print("state -> IDLE")

    def _handle_seat(self, command):
        if self.state != msg.MibSystemState.IDLE:
            print("ignoring seat command outside IDLE")
            return
        name = msg.SeatAxis(command.axis).name
        setattr(self.seat, {
            "FRONT_BACK_TILT": "front_back_tilt",
            "LATERAL_TILT": "lateral_tilt",
            "ELEVATION": "elevation",
            "TRANSLATION": "translation",
        }[name], command.target)
        print(f"seat {name} -> {command.target}")

    def _handle_param_set(self, request):
        ok = 0
        if 0 <= request.id < msg.PARAM_COUNT:
            _, name, _, unit, lo, hi, _ = msg.PARAM_TABLE[request.id]
            v = request.value
            if v == v and lo <= v <= hi:   # not NaN, in range
                self.params[request.id] = (1.0 if v >= 0.5 else 0.0) if unit == 'bool' else v
                ok = 1
                print(f"param {name} -> {self.params[request.id]:g} (seq {request.seq})")
        if not ok:
            print(f"param id {request.id} REJECTED value {request.value} (seq {request.seq})")
        self.last_set_seq = request.seq
        self.last_set_ok = ok

    def _handle_joystick(self, sample):
        if self.state == msg.MibSystemState.ENABLED:
            # The firmware maps y to linear and x to angular, with x's sign a parameter.
            self.target_linear = sample.y
            self.target_angular = sample.x * self.params[4]
        else:
            self.target_linear = self.target_angular = 0.0

    def _advance_motion(self, dt):
        base = 5 + 4 * self.profile   # four rows per profile, LOW first
        max_lin, max_ang, acc_lin, acc_ang = self.params[base:base + 4]
        target_lin = max(-max_lin, min(max_lin, self.target_linear))
        target_ang = max(-max_ang, min(max_ang, self.target_angular))

        def approach(current, target, rate):
            step = rate * dt
            if target > current:
                return min(current + step, target)
            return max(current - step, target)

        self.current_linear = approach(self.current_linear, target_lin, acc_lin)
        self.current_angular = approach(self.current_angular, target_ang, acc_ang)

    def _publish_status(self):
        self.seq = (self.seq + 1) & 0xFF
        status = msg.MibStatus(
            systemState=int(self.state),
            activeProfile=self.profile,
            currentSeatState=self.seat,
            error_message="No error",
            epoch_s=int(time.time()),
            speed=abs(self.current_linear),
            utc_offset_min=0,
            seq=self.seq,
            status_text="",
            error_footer="",
        )
        self.status_writer.write(status)

    def _publish_params(self):
        self.param_seq = (self.param_seq + 1) & 0xFF
        self.param_state_writer.write(msg.ParamState(
            seq=self.param_seq, last_set_seq=self.last_set_seq, last_set_ok=self.last_set_ok,
            count=msg.PARAM_COUNT, values=list(self.params)))

    def _publish_motor_commands(self):
        self.motor_seq = (self.motor_seq + 1) & 0xFF
        enabled = self.state == msg.MibSystemState.ENABLED
        if enabled:
            left_rpm, right_rpm = wheel_speeds(self.current_linear, self.current_angular, self.params)
        else:
            left_rpm = right_rpm = 0.0

        for axis, rpm in ((2, left_rpm), (3, right_rpm)):
            self.motor_writers[axis].write(msg.MotorCommand(
                seq=self.motor_seq,
                requested_state=int(
                    msg.RequestedState.ARMED if enabled else msg.RequestedState.DISARMED
                ),
                mode=int(msg.ControlMode.VELOCITY),
                velocity=rpm * RPM_TO_RAD_PER_S,
            ))


def main():
    parser = argparse.ArgumentParser(description="Stand-in for the RAMMP MIB firmware.")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id")
    parser.add_argument("--interface", default=None, help="network interface to bind")
    parser.add_argument("--boot-delay", type=float, default=1.0,
                        help="seconds to stay in INITIALIZING")
    parser.add_argument("--local", action="store_true",
                        help="use loopback, for running alongside the debugger")
    args = parser.parse_args()

    fake = FakeMib(args.domain, args.interface, args.boot_delay, args.local)

    def shutdown(*_):
        fake.running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    fake.start()
    try:
        fake.run()
    finally:
        print("fake MIB stopped")


if __name__ == "__main__":
    sys.exit(main() or 0)
