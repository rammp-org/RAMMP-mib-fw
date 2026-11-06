#!/usr/bin/env python3
"""Unprivileged DHCP test client for the MIB test build (DHCP on UDP 1067/1068, not real DHCP)."""
import socket, struct, random, time, sys, json
SRV=('192.168.1.255',1067); SRC='192.168.1.11'
def mk():
    r=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); r.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    r.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1); r.bind(('',1068))
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1); s.bind((SRC,0))
    return r,s
def ip(n): return socket.inet_aton(n)
def pkt(mac,xid,typ,req=None,ciaddr=None):
    h=struct.pack('!BBBBIHH',1,1,6,0,xid,0,0x8000)+(ip(ciaddr) if ciaddr else b'\0'*4)+b'\0'*12+mac+b'\0'*10+b'\0'*192
    o=b'\x63\x82\x53\x63'+bytes([53,1,typ])
    if req: o+=bytes([50,4])+ip(req)
    return (h+o+b'\xff').ljust(300,b'\0')
def parse(b):
    if len(b)<240: return None
    xid=struct.unpack('!I',b[4:8])[0]; y=socket.inet_ntoa(b[16:20]); opts={}; i=240
    while i<len(b) and b[i]!=255:
        if b[i]==0: i+=1; continue
        opts[b[i]]=b[i+2:i+2+b[i+1]]; i+=2+b[i+1]
    return dict(xid=xid,yiaddr=y,mtype=opts.get(53,b'\0')[0],opts=opts,chaddr=b[28:34])
def xfer(r,s,p,xid,want,timeout=1.0):
    s.sendto(p,SRV); end=time.time()+timeout; r.settimeout(timeout)
    while time.time()<end:
        try: b=r.recv(1500)
        except socket.timeout: break
        m=parse(b)
        if m and m['xid']==xid and m['mtype'] in want: return m
    return None
def dora(r,s,mac,timeout=1.0):
    xid=random.getrandbits(32); t=time.time()
    o=xfer(r,s,pkt(mac,xid,1),xid,(2,),timeout)
    if not o: return dict(ok=False,stage='no-offer',t=time.time()-t)
    a=xfer(r,s,pkt(mac,xid,3,req=o['yiaddr']),xid,(5,6),timeout)
    if not a: return dict(ok=False,stage='no-ack',t=time.time()-t,ip=o['yiaddr'])
    if a['mtype']==6: return dict(ok=False,stage='nak',t=time.time()-t,ip=o['yiaddr'])
    return dict(ok=True,ip=a['yiaddr'],t=time.time()-t,opts=a['opts'])
def macn(i): return bytes([2,0xd0,0,0,(i>>8)&255,i&255])
if __name__=='__main__':
    r,s=mk(); cmd=sys.argv[1]
    if cmd=='one':
        m=dora(r,s,macn(int(sys.argv[2]) if len(sys.argv)>2 else 1))
        if 'opts' in m:
            o=m.pop('opts'); m['router']=socket.inet_ntoa(o[3]) if 3 in o else None; m['dns']=socket.inet_ntoa(o[6]) if 6 in o else None
            m['lease_s']=struct.unpack('!I',o[51])[0] if 51 in o else None; m['optcodes']=sorted(o)
        print(json.dumps(m))
    elif cmd=='range':
        a,b=int(sys.argv[2]),int(sys.argv[3]); res={}
        for i in range(a,b):
            m=dora(r,s,macn(i)); print(i,m.get('stage','ok'),m.get('ip'),round(m['t'],3),flush=True)
