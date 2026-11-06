import sys,json,re
sys.path.insert(0,'/Users/alxchunlin/code-stuff/alex-style-guide/matplotlib')
import alex_style as ax
import matplotlib.pyplot as plt
import numpy as np
ax.use('dark'); c=ax.colors()
s1,s2=[c[k] for k in ('s1','s2')] if isinstance(c,dict) and 's1' in c else ('#2f62e8','#e03a2f')
B=json.load(open('/tmp/dhcp_fig/result_before.json')); A=json.load(open('/tmp/dhcp_fig/result_after.json'))
# Fig 1: where each restart hands out addresses
fig,a=plt.subplots(figsize=(8.6,4.4))
for R,col,name,dx in ((B,s1,'main',-0.13),(A,s2,'this PR',0.13)):
    xs=[r['boot']+dx for r in R['restarts'] for _ in r['got']]; ys=[v for r in R['restarts'] for v in r['got']]
    a.scatter(xs,ys,marker='s',s=20,color=col,label=name,linewidths=0)
a.set_xlabel('Restarts during the test, count'); a.set_ylabel('Address handed out, last octet')
a.set_xticks(range(7)); a.set_ylim(0,245)
a.legend(loc='upper left',bbox_to_anchor=(0,1),ncol=2,frameon=False,handletextpad=0.2,columnspacing=1.2)
ax.title(a,'Addresses given to 6 new clients on each boot','main repeats .2 to .7 every time. The windows do not repeat until the 6th restart')
ax.save(fig,'/tmp/dhcp_fig/out/dhcp_restart_addresses.png')
# Fig 2: leases lost
fig,a=plt.subplots(figsize=(8.6,3.1))
labs=['12','40']; yy=np.arange(2)
bv=[B['evict_12']['lost'],B['evict_40']['lost']]; av=[A['evict_12']['lost'],A['evict_40']['lost']]
h=0.36
for off,vals,col,name in ((-h/2,bv,s1,'main (keeps 8)'),(h/2,av,s2,'this PR (keeps 32)')):
    bars_=a.barh(yy+off,vals,height=h,color=col,label=name)
    for y,v in zip(yy+off,vals):
        a.text(v+0.15,y,str(v),va='center',family='Roboto Mono',color=c.get('ink','#eee') if isinstance(c,dict) else '#eee',fontsize=10)
a.set_yticks(yy); a.set_yticklabels(labs); a.invert_yaxis(); a.set_xlim(0,14)
a.set_xlabel('Clients that lost their lease within 12 s, count'); a.set_ylabel('Clients bound, count')
a.grid(axis='y',visible=False)
a.legend(loc='lower right',frameon=False)
ax.title(a,'Leases evicted without notice','Fresh boot, clients bound once, then idle for 12 s')
ax.save(fig,'/tmp/dhcp_fig/out/dhcp_leases_lost.png')
# Fig 3: heap in soak
def series(f):
    t=[];v=[]
    for l in open(f):
        m=re.search(r'NETDIAG up=(\d+) int_free=(\d+)',l)
        if m: t.append(int(m.group(1))/60); v.append(int(m.group(2))/1024)
    return t,v
fig,a=plt.subplots(figsize=(8.6,3.6))
for f,col,name in (('/tmp/dhcp_fig/sSbefore.log',s1,'main'),('/tmp/dhcp_fig/sSafter.log',s2,'this PR')):
    t,v=series(f); a.plot(t,v,color=col,lw=2,label=name); print(name,round(v[2],1),round(v[-1],1),round(min(v),1))
a.set_ylim(200,240); a.set_xlabel('Time since boot, min'); a.set_ylabel('Internal heap free, KB')
ax.label_lines(a); ax.title(a,'Free heap with 12 ROS 2 peers discovered','10 min each, no drive boards so no odometry data. y axis starts at 200 KB')
ax.save(fig,'/tmp/dhcp_fig/out/dhcp_heap_soak.png')
