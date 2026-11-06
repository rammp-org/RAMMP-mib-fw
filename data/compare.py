#!/usr/bin/env python3
"""One test script for both images. compare.py <label>; the same test build patch sits on each."""
import sys,json,time,subprocess,socket,struct,zlib,serial
sys.path.insert(0,'.'); import dhcpc
L=sys.argv[1]; MIB='192.168.1.250'; V='venv/bin/python'
R={'label':L}; r,s=dhcpc.mk(); ser=None
def sh(c): return subprocess.run(c,shell=True,capture_output=True,text=True).stdout
def stoplog():
    global ser
    if ser: ser.terminate(); ser.wait(); ser=None
def startlog(): 
    global ser; ser=subprocess.Popen([V,'serlog.py','s_%s.log'%L])
def reset():
    stoplog(); p=serial.Serial(); p.port='/dev/ttyACM0'; p.baudrate=115200; p.dtr=False; p.rts=True; p.open(); time.sleep(0.15); p.rts=False; time.sleep(0.05); p.close(); startlog()
def up(limit=25):
    t0=time.time()
    while time.time()-t0<limit:
        m=dhcpc.dora(r,s,dhcpc.macn(0x6fff),timeout=0.3)
        if m.get('ok'): return round(time.time()-t0,2)
    return None
def d(i): return dhcpc.dora(r,s,dhcpc.macn(i),timeout=0.5)
def octet(ip): return int(ip.split('.')[-1]) if ip else None
def status():
    f=struct.pack('<HBBBI',0x4F54,0x10,0x40,0x09,0); f+=struct.pack('<I',zlib.crc32(f))
    c=socket.create_connection((MIB,5555),timeout=3); c.sendall(f); b=b''; end=time.time()+3
    while time.time()<end:
        try: b+=c.recv(8192)
        except socket.timeout: break
        i=b.find(b'{"state"')
        if i>=0:
            j=b.rfind(b'}')
            try: return json.loads(b[i:j+1])
            except Exception: pass
    return None
# flash, clean NVS
sh('%s -m esptool --chip esp32p4 -p /dev/ttyACM0 -b 921600 erase-region 0x9000 0x6000'%V)
o=sh('%s -m esptool --chip esp32p4 -p /dev/ttyACM0 -b 921600 write-flash 0x2000 %s/bootloader.bin 0x8000 %s/partition-table.bin 0xf000 %s/ota_data_initial.bin 0x20000 %s/rammp_mib.bin'%(V,L,L,L,L))
print('flash ok' if 'verified' in o else o[-300:],flush=True)
startlog(); print('up',up(30),flush=True)
# eviction
for n,base in ((12,0x100),(40,0x200)):
    reset(); up(30); time.sleep(1)
    h={i:d(i).get('ip') for i in range(base,base+n)}
    time.sleep(12); kept=sum(d(i).get('ip')==ip for i,ip in h.items())
    R['evict_%d'%n]=dict(clients=n,kept=kept,lost=n-kept); print(R['evict_%d'%n],flush=True)
# restarts: every client keeps its address across all of them
held={}; R['restarts']=[]
for k in range(7):
    reset(); boot=up(30); time.sleep(1)
    got=[]; conflicts=0
    for j in range(6):
        ip=d(0x1000+k*16+j).get('ip'); got.append(octet(ip))
        if ip in held: conflicts+=1
    for j,o_ in enumerate(got): held['192.168.1.%s'%o_]=k
    ren=None
    if k:
        mac=0x1000+(k-1)*16; xid=9000+k
        ipr='192.168.1.%s'%R['restarts'][-1]['got'][0]
        m=dhcpc.xfer(r,s,dhcpc.pkt(dhcpc.macn(mac),xid,3,ciaddr=ipr),xid,(5,6),0.5); ren={None:'none seen',5:'ACK',6:'NAK'}[m and m['mtype']]
    R['restarts'].append(dict(boot=k,up_s=boot,got=got,conflicts=conflicts,renew=ren)); print(R['restarts'][-1],flush=True)
R['status_net']=(status() or {}).get('net'); print('status',R['status_net'],flush=True)
# boot time
b=[]
for k in range(15):
    reset(); b.append(up(25)); time.sleep(k%3)
R['boots']=b; print('boots',b,flush=True)
R['hwm']=[l for l in open('s_%s.log'%L) if 'evt_hwm' in l][-1].split('evt_hwm=')[-1].strip()
json.dump(R,open('result_%s.json'%L,'w'),indent=1)
stoplog(); print('done',flush=True)
