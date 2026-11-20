"""Read-only dump of nodes 3 and 2 (the configured ones) through the MIB's SDO_READ. SCAN is disabled in
the firmware (a probe client crashed the MIB), so the node list is fixed below."""
import os, sys, time, struct, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mibnet import Link
L=Link()
def scan(a,b):
    t,p=L.request(0x0F, bytes([a,b]), want=(0x8D,), timeout=15)
    if t!=0x8D: return []
    n=p[0]; return [(p[1+5*i], struct.unpack('<I',p[2+5*i:6+5*i])[0]) for i in range(n)]
def rd(node, idx, sub, nb=4):
    t,p=L.request(0x0E, bytes([node, idx&0xFF, idx>>8, sub, nb]), want=(0x8E,), timeout=3)
    if t==0x8E: return struct.unpack('<i', p[4:8])[0]
    return "err:"+(p[1:].decode(errors='replace') if p else "none")
found=[]
for a in []:
    found+=scan(a,min(a+9,127))
found=[(3,0),(2,0)]
out={}
for node,_ in found:
    d={"node":node}
    d["supported_modes_6502"]=rd(node,0x6502,0)
    d["heartbeat_consumer_1016_1"]=rd(node,0x1016,1)
    d["battery_dV?_2018"]=rd(node,0x2018,0,2)
    for ch,off in (("M1",0),("M2",0x800)):
        c={}
        c["position_6064"]=rd(node,0x6064+off,0)
        c["statusword_6041"]=rd(node,0x6041+off,0,2)
        c["window_6067"]=rd(node,0x6067+off,0)
        c["swlimit_607D"]=[rd(node,0x607D+off,1),rd(node,0x607D+off,2)]
        c["profile_vel_6081"]=rd(node,0x6081+off,0)
        pid=0x203F if ch=="M1" else 0x2040
        c["pos_pid_P_I_D_MaxI_db_min_max"]=[rd(node,pid,s) for s in range(1,8)]
        vp=0x2037 if ch=="M1" else 0x2038
        c["vel_pid_P_I_D_QPPS"]=[rd(node,vp,s) for s in range(1,5)]
        c["encoder_mode_205B"]=rd(node,0x205B,1 if ch=="M1" else 2,1)
        c["pwm_2030"]=rd(node,0x2030,1 if ch=="M1" else 2,2)
        c["current_x10mA_2031"]=rd(node,0x2031,1 if ch=="M1" else 2,2)
        c["fault_reaction_605E"]=rd(node,0x605E+off,0,2)
        d[ch]=c
    out[node]=d
print(json.dumps(out, indent=1))
json.dump(out, open('/tmp/newchair_dump.json','w'), indent=1)
