import math, numpy as np
def ik_ok(p, SB,SP,LUP,LLO, lo=-20, hi=70, tcp=0.0, remap=None):
    # firmware formulation (kinematics.cpp), probe z + tcp offset => effector centre
    x,y,z=p[0],p[1],p[2]-tcp
    s3=math.sqrt(3); wb=SB/(2*s3); up=SP/s3; wp=up/2
    a=wb-up; b=SP/2-(s3/2)*wb; c=wp-wb/2
    E=[2*LUP*(y+a), -LUP*(s3*(x+b)+y+c), LUP*(s3*(x-b)-y-c)]
    F=[2*z*LUP]*3
    r2=x*x+y*y+z*z
    G=[r2+a*a+LUP**2+2*y*a-LLO**2, r2+b*b+c*c+LUP**2+2*x*b+2*y*c-LLO**2, r2+b*b+c*c+LUP**2-2*x*b+2*y*c-LLO**2]
    th=[]
    for i in range(3):
        d=E[i]**2+F[i]**2-G[i]**2
        if d<0: return None
        den=G[i]-E[i]
        if abs(den)<1e-6: return None
        ok=False
        for sg in (1,-1):
            t=2*math.atan((-F[i]+sg*math.sqrt(d))/den)*180/math.pi
            if lo<=t<=hi: th.append(t); ok=True; break
        if not ok: return None
    return th
def onaxis(geom,tcp):
    zs=np.arange(-900,-400,0.5); good=[z for z in zs if ik_ok((0,0,z),*geom,tcp=tcp) is not None]
    return min(good),max(good)
def grid(geom,tcp):
    n=0;tot=0
    for x in np.arange(-150,150.1,25):
        for y in np.arange(-150,150.1,25):
            for z in np.arange(-790,-549,25):
                tot+=1
                if ik_ok((x,y,z),*geom,tcp=tcp) is not None: n+=1
    return n,tot
old=(175,150,180,625); new=(175,75,177,625)
for nm,g in (('HEAD config.h (SP150,L180)',old),('JSON calibrated (SP75,L177)',new)):
    for tcp in (0.0,-21.0):
        print(nm,'tcp',tcp,'on-axis z range %.1f..%.1f'%onaxis(g,tcp),'grid reachable %d/%d'%grid(g,tcp))

# zHome(0) (kinematics.cpp:91-95) for the HEAD and calibrated geometries
def zhome(SB, SP, L, LL, th=0.0):
    wb = SB / (2 * math.sqrt(3)); up = SP / math.sqrt(3); t = math.radians(th)
    r = wb + L * math.cos(t) - up
    return -L * math.sin(t) - math.sqrt(LL * LL - r * r)
print('zHome(0) HEAD %.1f  calibrated %.1f' % (zhome(*old), zhome(*new)))
