# MIB debugger: setup by platform

The debugger is a Python tool with one dependency, CycloneDDS. It runs on
macOS, Linux (including WSL) and Windows from the same files; only the way
Python is found and the network interface is named differ. This page is the
install reference. [TUTORIAL.md](TUTORIAL.md) is what to do once it runs, and
[README.md](README.md) describes every page.

## What every platform needs

- **CPython 3.9 or 3.10.** CycloneDDS 0.10.5 publishes prebuilt wheels for
  3.7 to 3.10 on macOS, Linux and Windows and nothing newer; a later Python
  makes pip try to compile the C library, which fails without a CycloneDDS
  install. The setup scripts look for a suitable interpreter and refuse to
  continue otherwise.
- **A wired Ethernet path to the MIB.** The MIB runs a DHCP server at
  192.168.4.1 and hands the computer a 192.168.4.x lease. A USB-C Ethernet
  adapter is fine. Wi-Fi does not reach the chair's network.
- **UDP in both directions on that interface**, unicast and multicast.
  Firewalls that block inbound UDP leave the tool on "searching" forever.
- Nothing is installed system-wide. Everything lives in a `.venv` folder next
  to the tool, so it cannot disturb a ROS 2 or other DDS installation on the
  same machine, and it sets its CycloneDDS configuration only inside its own
  process.

## macOS

```
cd tools/mib_debugger
./setup.sh
./run.sh
```

`setup.sh` prefers `/usr/bin/python3`, which is 3.9 on current macOS and has a
wheel. If it reports no suitable Python, `brew install python@3.10` and run it
again. A browser window opens at http://127.0.0.1:8712/.

The interface dropdown shows BSD names (`en9`); auto-detect picks the one
holding a 192.168.4.x address. macOS asks once whether Python may accept
incoming connections: allow it.

## Linux

```
cd tools/mib_debugger
./setup.sh
./run.sh
```

Ubuntu 22.04 ships Python 3.10 and works as is. Ubuntu 24.04 ships 3.12, which
has no wheel: install 3.10 beside it (`sudo add-apt-repository ppa:deadsnakes/ppa
&& sudo apt install python3.10 python3.10-venv`) and `setup.sh` finds it. The
interface list reads `ifconfig` (package `net-tools`) and falls back to
`ip -o -4 addr`, so either tool is enough. Interface names are the kernel's
(`eth0`, `enx…`). If `ufw` is on, allow UDP in on the board's interface.

### WSL

WSL2 sits behind NAT on a private virtual subnet, so by default it cannot see
the chair's network at all. Two ways to give it the link:

1. **Attach the USB Ethernet adapter to WSL** with
   [usbipd-win](https://github.com/dorssel/usbipd-win): on Windows,
   `usbipd list`, then `usbipd bind --busid <id>` and `usbipd attach --wsl
   --busid <id>`. The adapter leaves Windows and appears in WSL as a normal
   NIC; `sudo dhclient <iface>` takes the MIB's lease. Then follow the Linux
   steps above. This is the tested-equivalent path: the tool sees a real Linux
   interface.
2. **Mirrored networking** (Windows 11 22H2 or later): put
   `networkingMode=mirrored` under `[wsl2]` in `%UserProfile%\.wslconfig`,
   then `wsl --shutdown`. WSL shares the host's interfaces and multicast is
   supported in this mode. Less proven with DDS than option 1; if discovery
   works but no samples arrive, use option 1.

## Windows

From PowerShell or cmd (not by double-clicking, and the `.sh` files do nothing
there):

```
cd tools\mib_debugger
setup.cmd
run.cmd
```

`setup.cmd` uses the `py` launcher to find Python 3.10 or 3.9. If neither is
installed: `winget install Python.Python.3.10`, reopen the terminal, run
`setup.cmd` again. `run.cmd` and `sim.cmd` take the same arguments as the
`.sh` scripts, for example `run.cmd --local` with `sim.cmd` in a second
terminal for the simulator.

The interface dropdown shows the adapters' friendly names ("Ethernet 2") as
`ipconfig` reports them. The tool hands CycloneDDS the adapter's address
rather than its name, which is what it matches reliably on Windows;
`run.cmd --interface 192.168.4.4` works as well.

**Windows Firewall:** the first time Python receives from the network, a
prompt asks where to allow it. Tick both private and public networks. The
board's link is normally classed as a public network, and a private-only rule
drops its packets without any message, leaving the tool on "searching". To
fix it afterwards: Windows Defender Firewall, Allowed apps, find the Python
entry for the `.venv` interpreter and tick Public.

Git Bash users can keep using the `.sh` scripts; they find the venv's
`Scripts` folder on their own.

## Checking the link

1. Plug in, wait for the lease. On macOS or Linux `ifconfig` (or `ip addr`)
   shows a 192.168.4.x address; on Windows `ipconfig` does.
2. Start the tool, choose Board, leave the interface on Auto-detect, Connect.
3. The header pill turns green and the Board panel's sample count climbs
   within a couple of seconds. If the pill stays on "searching", see below.

## When it stays on "searching"

Run the doctor first. It performs the checks below in order and names the
first one that fails:

```
./doctor.sh                  # macOS, Linux; doctor.cmd on Windows
./doctor.sh --interface en9  # or an address
./doctor.sh --trace          # also summarises a CycloneDDS trace
```

Step 1 is the lease and a ping of the MIB, step 2 listens for the boards'
discovery multicast, step 3 joins the domain, lists every topic each board
offers and counts the samples that arrive. "MIB endpoints discovered" with
zero MibStatus samples is the endpoint-discovery handshake failing (the
boards never learned this machine's readers), not a cabling or firewall
problem; `--trace` shows whether the MIB ever acknowledged this machine's
discovery writers.

- **No lease.** The dropdown shows no entry marked "(board subnet)": cable,
  adapter or the MIB's DHCP. Ping 192.168.4.1.
- **Wrong interface.** With Wi-Fi up as well, pick the board's interface
  explicitly, Disconnect, Connect. CycloneDDS chooses its interface when the
  participant is created.
- **Firewall.** Inbound UDP blocked on the board's interface; see the platform
  notes above.
- **Managed switch with IGMP snooping and no querier.** The laptop's
  multicast reaches the boards but theirs never comes back. A dumb switch or
  a direct cable to the MIB avoids it.
- **Lines reading `malformed packet received from vendor 13.37`** in the
  terminal are noise, not the cause: CycloneDDS rejecting a pre-emptive
  ACKNACK the boards' RTPS stack sends with a sequence-number base of 0.
  Discovery completes on the next heartbeat regardless.
- **The MIB's boot log** over USB (`idf.py monitor`) shows whether its DDS
  endpoints were created; "BEST_EFFORT writer capacity reached" means a
  firmware build without the `CONFIG_RTPS_LIMIT_NUM_STATELESS_WRITERS=8`
  line in `sdkconfig.defaults`.
