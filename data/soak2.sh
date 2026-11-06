#!/bin/bash
# soak2.sh <label> <secs>: image already flashed. 12 peers + probes.
cd ~/mib_dhcp_test; V=venv/bin/python; T=$1; S=$2
$V rst.py; nohup $V serlog.py s$T.log >/dev/null 2>&1 &
sleep 12; cd ~/mib_dhcp_test
MIB=192.168.1.250 nohup $V probe2.py p$T.log 5 >/dev/null 2>&1 &
for i in $(seq 1 12); do nohup ~/mib_link_test/run python3 peers.py $i $S > peer$i.log 2>&1 & done
sleep $S; pkill -f probe2.py; pkill -f serlog.py; sleep 1; pkill -f peers.py; true
$V soakreport.py $T
