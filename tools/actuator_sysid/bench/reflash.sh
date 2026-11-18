#!/bin/zsh
# Close session.py (it sends `stop` first), flash the motion build, leave the board running.
# Restart session.py afterwards. Port: $SYSID_PORT. Run from anywhere.
set -e
HERE=${0:A:h}
PORT=${SYSID_PORT:-$(ls /dev/cu.usbmodem* | head -1)}
CMDS=${SYSID_CMDS:-$HERE/../.bench/cmds.txt}
PY=${IDF_PYTHON:-python3}
echo QUIT >> $CMDS; sleep 5.5
S=$(mktemp -d); mkdir -p $S/bootloader $S/partition_table
B=$HERE/../firmware/build_motion
cp $B/actuator_sysid.bin $B/ota_data_initial.bin $S/; cp $B/bootloader/bootloader.bin $S/bootloader/; cp $B/partition_table/partition-table.bin $S/partition_table/
cd $S   # esptool runs from a copy, so a rebuild cannot change the image mid-flash
$PY -m esptool --chip esp32p4 -p $PORT -b 460800 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 80m --flash-size 16MB 0x2000 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0xf000 ota_data_initial.bin 0x20000 actuator_sysid.bin | grep -E "Hard resetting|rror" | tail -3
: > $CMDS
