"""Read and change the MIB's drive profiles over RTPS.

    drive_profile.py show
    drive_profile.py set normal --max-linear-velocity-mps 1.2 [--persist]
    drive_profile.py set normal --from-json profile.json [--persist]
    drive_profile.py reset normal [--persist]

`show` prints every profile's limits as the MIB reports them on
rammp/mib/drive_profiles. `set` starts from the limits the MIB is using, changes
the fields given, and sends a DriveProfileCommand until the MIB acknowledges
it. Without --persist a change lasts until the MIB reboots; with it, `set`
saves the profile to the MIB's flash and `reset` deletes the saved copy.

The types mirror external/rammp-rtps (messages/mib_message.hpp and
messages/joystick_message.hpp): each struct is the XCDR1 wire layout, fields in
declaration order, and every C++ `enum class X : uint8_t` is a uint8 here.
"""

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field, fields
from enum import IntEnum

import cyclonedds.idl.types as pt
from cyclonedds.idl import IdlStruct

TOPIC_COMMAND = "rammp/joystick/drive_profile_command"
TOPIC_STATE = "rammp/mib/drive_profiles"
PROFILE_COUNT = 3

# The MIB publishes its profiles twice a second; allow several missed samples.
STATE_TIMEOUT_S = 3.0
ACK_TIMEOUT_S = 3.0
RESEND_PERIOD_S = 0.1


class DriveProfile(IntEnum):
    LOW = 0
    NORMAL = 1
    HIGH = 2


class DriveProfileAction(IntEnum):
    SET = 0
    RESET = 1


class DriveProfileSource(IntEnum):
    DEFAULT = 0
    SAVED = 1
    UNSAVED = 2


class DriveProfileResult(IntEnum):
    NONE = 0
    OK = 1
    INVALID = 2
    SAVE_FAILED = 3
    BAD_REQUEST = 4


@dataclass
class DriveProfileValues(IdlStruct, typename="rammp/msg/DriveProfileValues"):
    max_linear_velocity_mps: pt.float32 = 0.0
    max_reverse_linear_velocity_mps: pt.float32 = 0.0
    max_angular_velocity_radps: pt.float32 = 0.0
    max_linear_acceleration_mps2: pt.float32 = 0.0
    max_angular_acceleration_radps2: pt.float32 = 0.0
    max_linear_jerk_mps3: pt.float32 = 0.0
    max_angular_jerk_radps3: pt.float32 = 0.0
    stopping_linear_acceleration_mps2: pt.float32 = 0.0
    stopping_angular_acceleration_radps2: pt.float32 = 0.0
    max_centripetal_acceleration_mps2: pt.float32 = 0.0


@dataclass
class DriveProfileCommand(IdlStruct, typename="rammp/msg/DriveProfileCommand"):
    req_id: pt.uint8 = 0
    profile: pt.uint8 = DriveProfile.NORMAL
    action: pt.uint8 = DriveProfileAction.SET
    persist: pt.uint8 = 0
    values: DriveProfileValues = field(default_factory=DriveProfileValues)


@dataclass
class DriveProfileState(IdlStruct, typename="rammp/msg/DriveProfileState"):
    seq: pt.uint8 = 0
    command_ack: pt.uint8 = 0
    result: pt.uint8 = DriveProfileResult.NONE
    reserved0: pt.uint8 = 0
    sources: pt.array[pt.uint8, PROFILE_COUNT] = field(
        default_factory=lambda: [0] * PROFILE_COUNT)
    reserved1: pt.uint8 = 0
    profiles: pt.array[DriveProfileValues, PROFILE_COUNT] = field(
        default_factory=lambda: [DriveProfileValues() for _ in range(PROFILE_COUNT)])


VALUE_FIELDS = [f.name for f in fields(DriveProfileValues)]


def cyclonedds_uri(interface):
    """CycloneDDS config that sends discovery out of `interface`.

    On a laptop with Wi-Fi up as well, CycloneDDS may pick the wrong interface
    and never see the MIB, so name the one facing the board.
    """
    return (
        "<CycloneDDS><Domain><General><Interfaces>"
        f'<NetworkInterface name="{interface}" priority="default" multicast="true"/>'
        "</Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"
    )


class Link:
    """One participant, the command writer, and the state reader."""

    def __init__(self, domain_id, interface):
        if interface:
            os.environ["CYCLONEDDS_URI"] = cyclonedds_uri(interface)
        from cyclonedds.core import Policy, Qos
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic

        # The MIB's endpoints are best-effort with no durability; match them.
        qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(8))
        self.participant = DomainParticipant(domain_id)
        self.writer = DataWriter(
            self.participant, Topic(self.participant, TOPIC_COMMAND, DriveProfileCommand, qos=qos),
            qos=qos)
        self.reader = DataReader(
            self.participant, Topic(self.participant, TOPIC_STATE, DriveProfileState, qos=qos),
            qos=qos)

    def latest_state(self, timeout_s=STATE_TIMEOUT_S, newer_than=None):
        """The newest DriveProfileState, waiting up to timeout_s for one."""
        deadline = time.monotonic() + timeout_s
        latest = None
        while time.monotonic() < deadline:
            for sample in self.reader.take(N=16):
                if isinstance(sample, DriveProfileState):
                    latest = sample
            if latest is not None and (newer_than is None or newer_than(latest)):
                return latest
            time.sleep(0.02)
        return latest if newer_than is None else None

    def send(self, command):
        """Send until the MIB acknowledges command.req_id; return its state then."""
        deadline = time.monotonic() + ACK_TIMEOUT_S
        while time.monotonic() < deadline:
            self.writer.write(command)
            state = self.latest_state(RESEND_PERIOD_S,
                                      newer_than=lambda s: s.command_ack == command.req_id)
            if state is not None:
                return state
        return None


def print_state(state):
    print(f"last command: ack {state.command_ack}, "
          f"result {DriveProfileResult(state.result).name}")
    names = [p.name for p in DriveProfile]
    width = max(len(n) for n in VALUE_FIELDS)
    print(f"{'':{width}}  " + "  ".join(f"{n:>9}" for n in names))
    print(f"{'source':{width}}  " + "  ".join(
        f"{DriveProfileSource(s).name:>9}" for s in state.sources))
    for name in VALUE_FIELDS:
        row = [getattr(p, name) for p in state.profiles]
        print(f"{name:{width}}  " + "  ".join(f"{v:9.3f}" for v in row))


def next_req_id(state):
    """A req_id the MIB has not just acted on: one past its last ack, skipping 0."""
    return state.command_ack % 255 + 1


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", help="network interface facing the MIB, e.g. en7")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (default 0)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("show", help="print every profile's limits")

    profile_names = [p.name.lower() for p in DriveProfile]
    set_parser = sub.add_parser("set", help="change one profile's limits")
    set_parser.add_argument("profile", choices=profile_names)
    set_parser.add_argument("--persist", action="store_true", help="also save to the MIB's flash")
    set_parser.add_argument("--from-json", metavar="FILE",
                            help="read limits from a JSON object keyed by field name")
    for name in VALUE_FIELDS:
        set_parser.add_argument("--" + name.replace("_", "-"), type=float, dest=name)

    reset_parser = sub.add_parser("reset", help="go back to the compiled-in limits")
    reset_parser.add_argument("profile", choices=profile_names)
    reset_parser.add_argument("--persist", action="store_true",
                              help="also delete the profile saved in the MIB's flash")

    args = parser.parse_args()
    link = Link(args.domain, args.interface)

    state = link.latest_state()
    if state is None:
        sys.exit(f"No DriveProfileState on {TOPIC_STATE} within {STATE_TIMEOUT_S} s. "
                 "Is the MIB up, and is --interface the one facing it?")
    if args.command == "show":
        print_state(state)
        return

    profile = DriveProfile[args.profile.upper()]
    command = DriveProfileCommand(req_id=next_req_id(state), profile=profile,
                                  persist=1 if args.persist else 0)
    if args.command == "set":
        values = state.profiles[profile]
        changes = {}
        if args.from_json:
            with open(args.from_json) as f:
                changes.update(json.load(f))
        changes.update({n: getattr(args, n) for n in VALUE_FIELDS if getattr(args, n) is not None})
        unknown = set(changes) - set(VALUE_FIELDS)
        if unknown:
            sys.exit(f"Unknown field(s): {', '.join(sorted(unknown))}")
        if not changes:
            sys.exit("Nothing to change: give at least one field, or --from-json.")
        for name, value in changes.items():
            setattr(values, name, float(value))
        command.action = DriveProfileAction.SET
        command.values = values
    else:
        command.action = DriveProfileAction.RESET

    reply = link.send(command)
    if reply is None:
        sys.exit(f"The MIB did not acknowledge req_id {command.req_id} within {ACK_TIMEOUT_S} s.")
    print_state(reply)
    if reply.result != DriveProfileResult.OK:
        sys.exit(1)


if __name__ == "__main__":
    main()
