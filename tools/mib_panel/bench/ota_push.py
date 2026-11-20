"""Flash a rammp_mib image to the MIB over Ethernet (the panel Firmware tab's BEGIN/DATA/FINISH), then
read back its version.  ota_push.py <path/to/rammp_mib.bin>"""
import os, sys, time, struct, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mibnet import Link
img=open(sys.argv[1],'rb').read(); assert img[0]==0xE9
print("image", img[48:80].split(b'\0')[0].decode(), img[80:112].split(b'\0')[0].decode(), len(img))
L=Link()
def ota(t, payload=b"", timeout=5):
    rt,p=L.request(t, payload, want=(5,6,11), mod=0, timeout=timeout)
    if rt==6: raise SystemExit(f"OTA error {struct.unpack('<I',p[:4])[0]}: {p[4:].decode(errors='replace')}")
    if rt is None: raise SystemExit(f"OTA {t}: no reply")
    return rt,p
ota(1, struct.pack('<I',len(img)), 60)
for off in range(0,len(img),4096): ota(2, img[off:off+4096], 5)
ota(3, b"", 60); print("written; rebooting")
time.sleep(5)
for i in range(30):
    try: L=Link(); break
    except Exception: time.sleep(2)
t,p=L.request(2, want=(0x82,), mod=0x20); info=json.loads(p); print({k:info[k] for k in ('version','partition','pending_verify','uptime_s','reset_reason')})
