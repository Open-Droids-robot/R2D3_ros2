#!/usr/bin/env python3
import sys, numpy as np, xml.etree.ElementTree as ET
def R(rpy):
    r,p,y=rpy; cx,sx=np.cos(r),np.sin(r); cy,sy=np.cos(p),np.sin(p); cz,sz=np.cos(y),np.sin(y)
    return (np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
            @ np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
            @ np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]]))
def parse(path):
    root=ET.parse(path).getroot(); J={}
    for j in root.findall('joint'):
        c=j.find('child'); o=j.find('origin')
        if c is None: continue
        xyz=[float(v) for v in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()]
        rpy=[float(v) for v in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()]
        p=j.find('parent')
        J[c.get('link')]=(p.get('link') if p is not None else None, np.array(xyz), R(rpy))
    return J
def chain(J, frame):
    T=np.eye(4)
    while frame in J and J[frame][0] is not None:
        parent,t,Rm=J[frame]
        M=np.eye(4); M[:3,:3]=Rm; M[:3,3]=t
        T=M@T; frame=parent
    return T, frame
path, child = sys.argv[1], sys.argv[2]
J=parse(path); T,root=chain(J, child)
exp_t=np.array([float(v) for v in sys.argv[3].split()])
exp_R=R([float(v) for v in sys.argv[4].split()])
ok = root=='base_footprint' and np.allclose(T[:3,3],exp_t,atol=1e-4) and np.allclose(T[:3,:3],exp_R,atol=1e-4)
print("root=",root," t=",np.round(T[:3,3],4))
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
