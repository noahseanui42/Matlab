import csv, numpy as np
from pathlib import Path
D=str(Path(__file__).resolve().parents[1]/'FieldScan'/'data')+'/'
def ld(f):
    r=list(csv.DictReader(open(D+f)))
    g=lambda k:np.array([float(x[k]) for x in r])
    return dict(P=np.c_[g('x_mm'),g('y_mm'),g('z_mm')],S=np.c_[g('sent_x_mm'),g('sent_y_mm'),g('sent_z_mm')],B=np.c_[g('Bx_G'),g('By_G'),g('Bz_G')]*1e5)  # nT
off=[ld('2026-10-02_dryrun1_magnet_20261002_180000.csv'),ld('2026-10-02_dryrun1_magnet_off2_20261002_184415.csv')]
hyb=[ld('2026-10-02_dryrun1_magnet_hybrid_20261002_182828.csv'),ld('2026-10-02_dryrun1_magnet_hybrid2_20261002_185818.csv')]
P=off[0]['P']
xs,ys,zs=[np.unique(P[:,i]) for i in range(3)]
ix=np.searchsorted(xs,P[:,0]);iy=np.searchsorted(ys,P[:,1]);iz=np.searchsorted(zs,P[:,2])
def grad_at(B):
    G=np.zeros((len(P),3,3))  # G[p,comp,axis]
    for c in range(3):
        V=np.zeros((len(xs),len(ys),len(zs))); V[ix,iy,iz]=B[:,c]
        gx,gy,gz=np.gradient(V,xs,ys,zs)
        G[:,c,0]=gx[ix,iy,iz];G[:,c,1]=gy[ix,iy,iz];G[:,c,2]=gz[ix,iy,iz]
    return G
def test(Boff,Bhyb,e,label):
    G=grad_at(Boff)
    pred=np.einsum('pca,pa->pc',G,e)
    meas=Bhyb-Boff
    pv,mv=pred.ravel(),meas.ravel()
    k=(pv@mv)/(pv@pv); r=np.corrcoef(pv,mv)[0,1]
    res=mv-k*pv
    print('%-34s n=%d k=%.2f r=%.2f rms resid %.0f nT  rms meas %.0f nT  rms pred %.0f nT'%(label,len(pv),k,r,np.sqrt(np.mean(res**2)),np.sqrt(np.mean(mv**2)),np.sqrt(np.mean(pv**2))))
e1=hyb[0]['S']-hyb[0]['P']; e2=hyb[1]['S']-hyb[1]['P']
test(off[0]['B'],hyb[0]['B'],e1,'off1 -> hybrid1')
test(off[1]['B'],hyb[1]['B'],e2,'off2 -> hybrid2')
Boff=np.mean([o['B'] for o in off],0);Bhyb=np.mean([h['B'] for h in hyb],0)
test(Boff,Bhyb,(e1+e2)/2,'mean(off) -> mean(hybrid)')
# run-to-run scatter off1 vs off2, hyb1 vs hyb2 (rms of diff/sqrt2 and of diff)
d=(off[0]['B']-off[1]['B']).ravel(); d2=(hyb[0]['B']-hyb[1]['B']).ravel()
print('rms(off1-off2)=%.0f nT, rms(hyb1-hyb2)=%.0f nT  -> per-run scatter ~ %.0f / %.0f nT'%(np.sqrt(np.mean(d**2)),np.sqrt(np.mean(d2**2)),np.sqrt(np.mean(d**2))/np.sqrt(2),np.sqrt(np.mean(d2**2))/np.sqrt(2)))
# no-magnet background subtracted variant
bg=ld('2026-10-02_dryrun1_no_magnet.csv')['B']
test(off[0]['B']-bg,hyb[0]['B']-bg,e1,'(bg-subtracted) off1->hyb1')
# field at (50,0,-675) and (-50,0,-675)
for nm,sel in (('+50,0,-675',(50,0,-675)),('-50,0,-675',(-50,0,-675)),('+50,0,-650',(50,0,-650))):
    i=np.where((P==sel).all(1))[0][0]
    for k_,(o,bb) in enumerate(zip(off,[bg,bg])):
        print(nm,'off%d magnet-only |B| = %.1f uT'%(k_+1,np.linalg.norm(o['B'][i]-bg[i])/1000))
