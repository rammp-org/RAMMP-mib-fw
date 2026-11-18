"""Where the bench scripts find the MIB and the session they share.

    SYSID_PORT  serial port of the MIB console (default: the only /dev/cu.usbmodem* present)
    SYSID_LOG   log file session.py appends to and the other scripts read (required)
    SYSID_CMDS  command file session.py watches (default: tools/actuator_sysid/.bench/cmds.txt)
"""
import glob, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
CMDS = os.environ.get("SYSID_CMDS", os.path.join(HERE, "..", ".bench", "cmds.txt"))


def port():
    if os.environ.get("SYSID_PORT"):
        return os.environ["SYSID_PORT"]
    found = sorted(glob.glob("/dev/cu.usbmodem*"))
    if len(found) != 1:
        sys.exit(f"set SYSID_PORT: found {found or 'no'} /dev/cu.usbmodem* ports")
    return found[0]


def log():
    if not os.environ.get("SYSID_LOG"):
        sys.exit("set SYSID_LOG to the session log file (the one session.py writes)")
    return os.environ["SYSID_LOG"]


def say(command):
    """Queue one console command for session.py."""
    os.makedirs(os.path.dirname(CMDS), exist_ok=True)
    with open(CMDS, "a") as f:
        f.write(command + "\n")


def script(name):
    return os.path.join(HERE, name)
