#!/usr/bin/env python3
"""Serial console logger: keeps NETDIAG and anything alarming, with wall-clock stamps."""
import serial,sys,time,re
out=open(sys.argv[1],'a',buffering=1)
s=serial.Serial(); s.port='/dev/ttyACM0'; s.baudrate=115200; s.timeout=0.5; s.dtr=False; s.rts=False; s.open()
keep=re.compile(rb'NETDIAG|Guru|panic|abort|assert|Backtrace|rst:|Rebooting|boot:|Link (up|down)|DHCP|lease|Failed|stack|E \(|/W\]|heap',re.I)
drop=re.compile(rb'placeholder')
buf=b''; last=0
while True:
    buf+=s.read(8192)
    *ls,buf=buf.split(b'\n')
    for l in ls:
        if keep.search(l) and not drop.search(l):
            out.write(time.strftime('%H:%M:%S ')+re.sub(rb'\x1b\[[0-9;]*m',b'',l).decode(errors='replace').rstrip()+'\n')
