import csv, numpy as np
from pathlib import Path
D=str(Path(__file__).resolve().parents[1]/'FieldScan'/'data')+'/'
def ld(f):
    r=list(csv.DictReader(open(D+f)))
    g=lambda k:np.array([float(x[k]) for x in r])
    return np.c_[g('x_mm'),g('y_mm'),g('z_mm')],np.c_[g('Bx_G'),g('By_G'),g('Bz_G')],np.c_[g('Bx_std_G'),g('By_std_G'),g('Bz_std_G')],g('n_samples')
def compare(files,noise):
    P=None;M=[]
    for f in files:
        p,b,_,_=ld(f); P=p if P is None else P; M.append(np.linalg.norm(b,axis=1))
    M=np.array(M).T
    Bmean=np.mean([ld(f)[1] for f in files],axis=0); Bm=np.linalg.norm(Bmean,axis=1)
    sig=M.std(axis=1,ddof=1)
    p0,B0,S0,n0=ld(noise); sd0=S0/np.sqrt(np.maximum(n0,1))[:,None]; u0=B0/np.maximum(np.linalg.norm(B0,axis=1),1e-12)[:,None]
    sn=np.sqrt(((u0*sd0)**2).sum(1)); sc=np.sqrt(np.maximum(sig**2-sn**2,0))
    xs,ys,zs=[np.unique(np.round(P[:,i],2)) for i in range(3)]
    V=np.full((len(xs),len(ys),len(zs)),np.nan)
    ix=np.searchsorted(xs,np.round(P[:,0],2));iy=np.searchsorted(ys,np.round(P[:,1],2));iz=np.searchsorted(zs,np.round(P[:,2],2))
    V[ix,iy,iz]=Bm
    gx,gy,gz=np.gradient(V,xs,ys,zs); g=np.sqrt(gx**2+gy**2+gz**2)[ix,iy,iz]
    sp=sc/g; weak=~(g>0.1*np.nanmedian(g)); sp[weak]=np.nan
    return dict(n=len(P),usable=int(np.sum(~weak&np.isfinite(sp))),sigB_med_nT=np.median(sig)*1e5/100*100 if False else np.median(sig)*1e5,sigB_max_nT=sig.max()*1e5,noise_med_nT=np.median(sn)*1e5,sigpos_med=np.nanmedian(sp),sigpos_p95=np.nanpercentile(sp,95),sigpos_max=np.nanmax(sp))
nm='2026-10-02_dryrun1_no_magnet.csv'
off=['2026-10-02_dryrun1_magnet_20261002_180000.csv','2026-10-02_dryrun1_magnet_off2_20261002_184415.csv']
hyb=['2026-10-02_dryrun1_magnet_hybrid_20261002_182828.csv','2026-10-02_dryrun1_magnet_hybrid2_20261002_185818.csv']
for nme,fs in (('off x2',off),('hybrid x2',hyb)):
    print(nme,{k:(round(v,3) if isinstance(v,float) else v) for k,v in compare(fs,nm).items()})
# zeros
import glob
for f in ['2026-10-02_dryrun1_no_magnet.csv']+off+hyb:
    r=list(csv.DictReader(open(D+f)))
    z=sum(1 for x in r for k in ('Bx_G','By_G','Bz_G') if float(x[k])==0.0)
    zs=sum(1 for x in r for k in ('Bx_std_G','By_std_G','Bz_std_G') if float(x[k])==0.0)
    print(f,'exact-zero mean components:',z,'of',3*len(r),'; zero-std components',zs)
