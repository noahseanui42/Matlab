import csv, glob, json, numpy as np, os
from pathlib import Path
D=str(Path(__file__).resolve().parents[1]/'FieldScan'/'data')+'/'
def load(f):
    r=list(csv.DictReader(open(D+f)))
    a=lambda k: np.array([float(x['%s'%k]) if False else float(x[k]) for x in r])
    P=np.c_[a('x_mm'),a('y_mm'),a('z_mm')]
    B=np.c_[a('Bx_G'),a('By_G'),a('Bz_G')]*100  # uT
    S=np.c_[a('Bx_std_G'),a('By_std_G'),a('Bz_std_G')]*100
    return P,B,S,np.array([int(x['err']) for x in r]),np.array([float(x['t_s']) for x in r]),r
runs={'no_magnet':'2026-10-02_dryrun1_no_magnet.csv','off1':'2026-10-02_dryrun1_magnet_20261002_180000.csv','off2':'2026-10-02_dryrun1_magnet_off2_20261002_184415.csv','hyb1':'2026-10-02_dryrun1_magnet_hybrid_20261002_182828.csv','hyb2':'2026-10-02_dryrun1_magnet_hybrid2_20261002_185818.csv','close_a':'mag_off_1_20261002_024004.csv','close_b':'mag_off_1_20261002_031036.csv','s05':'settle_0p5s/mag_off_1_20261002_022713.csv'}
L={k:load(v) for k,v in runs.items()}
for k,(P,B,S,e,t,r) in L.items():
    mag=np.linalg.norm(B,axis=1)
    print(k,'n=%d err!=0:%d'%(len(P),(e!=0).sum()),'|B| uT min/med/max %.1f/%.1f/%.1f'%(mag.min(),np.median(mag),mag.max()),
      'std nT med/max %.0f/%.0f'%(np.median(S)*1000,S.max()*1000), 'T=%.0fs  %.1f s/pt'%(t[-1],t[-1]/len(P)), 'zeros:',int((B==0).sum()))
# per-axis std medians
P0,B0=L['no_magnet'][0],L['no_magnet'][1]
print('no_magnet B mean',B0.mean(0),'|B| range',np.linalg.norm(B0,axis=1).min(),np.linalg.norm(B0,axis=1).max())
# differences (match by position)
def diff(a,b):
    return L[a][1]-L[b][1]
def grad_mag(Bm,P):
    # |B| magnitude gradient on 5x5x3 grid (x fastest within serpentine) -> reshape by sorting
    xs=np.unique(P[:,0]);ys=np.unique(P[:,1]);zs=np.unique(P[:,2])
    V=np.zeros((len(xs),len(ys),len(zs)))
    for p,m in zip(P,Bm):
        V[np.searchsorted(xs,p[0]),np.searchsorted(ys,p[1]),np.searchsorted(zs,p[2])]=m
    gx,gy,gz=np.gradient(V,xs,ys,zs)
    G=np.zeros(len(P))
    for i,p in enumerate(P):
        ix,iy,iz=np.searchsorted(xs,p[0]),np.searchsorted(ys,p[1]),np.searchsorted(zs,p[2])
        G[i]=np.sqrt(gx[ix,iy,iz]**2+gy[ix,iy,iz]**2+gz[ix,iy,iz]**2)
    return G
P=L['off1'][0]
bg=L['no_magnet'][1]
for name in ['off1','off2','hyb1','hyb2']:
    Bm=np.linalg.norm(L[name][1]-bg,axis=1)
    print(name,'magnet-only |B| uT min/med/max %.1f/%.1f/%.1f'%(Bm.min(),np.median(Bm),Bm.max()))
# pairwise spreads of |B| (sensor-frame, baseline-subtracted)
import itertools
def sd_run(names):
    M=np.array([np.linalg.norm(L[n][1]-bg,axis=1) for n in names])
    return M.std(axis=0,ddof=1)
for names in (['off1','off2'],['hyb1','hyb2'],['off1','off2','hyb1','hyb2']):
    s=sd_run(names)
    G=grad_mag(np.mean([np.linalg.norm(L[n][1]-bg,axis=1) for n in names],axis=0),P)
    pos=s/np.where(G>0,G,np.nan)  # uT / (uT/mm)
    print(names,'sigma|B| nT med/p95 %.0f/%.0f'%(np.median(s)*1000,np.percentile(s,95)*1000),' median grad uT/mm %.3f'%np.median(G),'sigma_pos mm med %.2f'%np.nanmedian(pos))
# vector difference magnitude
for a,b in [('off1','off2'),('hyb1','hyb2'),('off1','hyb1'),('off2','hyb2'),('off1','hyb2'),('close_a','close_b')]:
    d=np.linalg.norm(L[a][1]-L[b][1],axis=1)
    print(a,b,'|dB| uT med/max %.2f/%.2f'%(np.median(d),d.max()))
# uniform-offset-removed vector spread (drift removal) off1 vs off2
d=L['off1'][1]-L['off2'][1]; d0=d-d.mean(0)
print('off1-off2 mean offset (uT)',d.mean(0),' residual |.| med %.3f uT'%np.median(np.linalg.norm(d0,axis=1)))
d=L['hyb1'][1]-L['hyb2'][1]; d0=d-d.mean(0); print('hyb1-hyb2 mean offset',d.mean(0),' resid med %.3f'%np.median(np.linalg.norm(d0,axis=1)))
# settle 0.5 vs 5 s: compare s05 vs close_a? they are different magnet placement? print notes
for k in ('s05','close_a','close_b'):
    print(k, json.load(open(D+runs[k].replace('.csv','.meta.json')))['note'])
d=np.linalg.norm(L['s05'][1]-L['close_a'][1],axis=1); print('s05 vs close_a |dB| uT med/max %.2f/%.2f'%(np.median(d),d.max()))
d=np.linalg.norm(L['s05'][1]-L['close_b'][1],axis=1); print('s05 vs close_b |dB| uT med/max %.2f/%.2f'%(np.median(d),d.max()))
# hybrid sent vs target offsets
r=L['hyb1'][5]
dx=np.array([[float(x['sent_x_mm'])-float(x['x_mm']),float(x['sent_y_mm'])-float(x['y_mm']),float(x['sent_z_mm'])-float(x['z_mm'])] for x in r])
print('hybrid sent-target (mm): max|dx|,|dy|,|dz|',np.abs(dx).max(0),' max norm %.1f'%np.linalg.norm(dx,axis=1).max())
for x in r:
    if abs(float(x['x_mm']))==50 and abs(float(x['y_mm']))==50 and float(x['z_mm'])==-650: pass
# the corner rows
for x in r:
    if abs(float(x['x_mm']))==50 and abs(float(x['y_mm']))==50: print(x['x_mm'],x['y_mm'],x['z_mm'],'->',x['sent_x_mm'],x['sent_y_mm'],x['sent_z_mm'])
