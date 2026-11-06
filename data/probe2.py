#!/usr/bin/env python3
"""Every N s: DORA a fixed fake MAC, ping the MIB, connect tcp :5555. Logs each sample."""
import socket,subprocess,sys,time,json
sys.path.insert(0,'.'); import dhcpc
log=open(sys.argv[1],'a',buffering=1); period=float(sys.argv[2]) if len(sys.argv)>2 else 5
r,s=dhcpc.mk(); import os; MIB=os.environ.get('MIB','192.168.1.77'); prev=None
while True:
    t=time.time(); m=dhcpc.dora(r,s,dhcpc.macn(0x7777),timeout=1.0)
    p=subprocess.run(['ping','-I','eno1','-c1','-W1',MIB],capture_output=True).returncode==0
    c=socket.socket(); c.settimeout(1); 
    try: c.connect((MIB,5555)); tcp=True
    except Exception: tcp=False
    c.close()
    st=(m.get('ok'),m.get('stage'),p,tcp)
    log.write(json.dumps(dict(t=time.strftime('%H:%M:%S'),dhcp=m.get('stage','ok'),ip=m.get('ip'),ms=round(m['t']*1000),ping=p,tcp=tcp))+'\n')
    time.sleep(max(0,period-(time.time()-t)))
