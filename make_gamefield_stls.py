import trimesh,numpy as np
from shapely.geometry import box,Polygon
from shapely.ops import unary_union
m=trimesh.load('/tmp/step_mesh.stl')
b=m.split(only_watertight=False)
main=b[5]
# origin: XY centre of outer field boundary, Z=0 at floor level (-740)
lo,hi=main.bounds
cx,cy=(lo[0]+hi[0])/2,(lo[1]+hi[1])/2
FLOOR_Z=-740.0
T=np.array([-cx,-cy,-FLOOR_Z])
print('shift',T)
def shifted(mesh):
    mm=mesh.copy(); mm.apply_translation(T); return mm
floor_tris=[];wall_tris=[]
platforms=[b[6],b[7]]
from shapely.geometry import MultiPoint
plat_fp=[MultiPoint(shifted(p).vertices[:,:2]).convex_hull for p in platforms]
plat_fp=unary_union(plat_fp)
for i,body in enumerate(b):
    s=shifted(body); n=s.face_normals; t=s.triangles
    zmax=t[:,:,2].max(1); zmin=t[:,:,2].min(1)
    vert=abs(n[:,2])<0.02
    up=n[:,2]>0.02
    if i in (6,7):
        V=s.vertices
        top=V[np.isclose(V[:,2],V[:,2].max(),atol=0.5)]   # 4 corners of the true +50 mm top
        slopes=n[:,2]>0.9
        slopes&=n[:,2]<0.999                                 # the two slanted ramp faces
        floor_tris.append(t[slopes])
        # fill the 20 mm depression: one flush flat top at full ramp height (AB)
        c=top[:,:2].mean(0); order=np.argsort(np.arctan2(top[:,1]-c[1],top[:,0]-c[0]))
        q=top[order]; q[:,2]=V[:,2].max()
        floor_tris.append(np.array([[q[0],q[1],q[2]],[q[0],q[2],q[3]]]))
        # rebuild the two 50 mm flanks as clean trapezoids (original flanks carry the notch)
        hi=V[:,2]>V[:,2].max()-0.5; lo=V[:,2]<0.5
        B=V[lo]; Tt=V[hi]
        cen=V.mean(0)
        for sign in (1,-1):
            bb=B[(B[:,1]-cen[1])*sign>0]; tt=Tt[(Tt[:,1]-cen[1])*sign>0]
            bb=bb[np.argsort(bb[:,0])]; tt=tt[np.argsort(tt[:,0])]
            quad=np.array([bb[0],bb[-1],tt[-1],tt[0]])
            tri=np.array([[quad[0],quad[1],quad[2]],[quad[0],quad[2],quad[3]]])
            nn=np.cross(tri[0,1]-tri[0,0],tri[0,2]-tri[0,0])
            if np.dot(nn,[0,sign,0])<0: tri=tri[:,::-1]
            floor_tris.append(tri)
        continue
    if i==5:
        sel=(n[:,2]>0.99)&(zmax<=5)                   # base floor (handled below)
        base=t[sel]
    elif i==0:
        floor_tris.append(t[up&(zmax<=30)&(zmin>=-1)])  # half-round bumps on floor
    wall_tris.append(t[vert|((abs(n[:,2])>0.99)&(zmin>60))])   # vertical faces + horizontal faces on walls (tops, ledges)
base=np.array(base)
# base floor: union of planar triangles, snap to z=0, remove platform footprints, retriangulate
polys=[Polygon(tr[:,:2]) for tr in base if abs(Polygon(tr[:,:2]).area)>1e-6]
poly=unary_union([p.buffer(0.01) for p in polys]).buffer(-0.01)
poly=poly.difference(plat_fp)
geoms=list(poly.geoms) if hasattr(poly,'geoms') else [poly]
fl=[]
for g in geoms:
    if g.area<1: continue
    v,f=trimesh.creation.triangulate_polygon(g,engine='earcut')
    fl.append(np.c_[v,np.zeros(len(v))][f])
floor_tris.append(np.concatenate(fl))
floor=trimesh.Trimesh(np.concatenate([x.reshape(-1,3) for x in floor_tris]),
        np.arange(sum(len(x) for x in floor_tris)*3).reshape(-1,3),process=False)
floor.merge_vertices(); 
walls=trimesh.Trimesh(np.concatenate([x.reshape(-1,3) for x in wall_tris]),
        np.arange(sum(len(x) for x in wall_tris)*3).reshape(-1,3),process=False)
walls=walls.slice_plane([0,0,0],[0,0,1],cap=False)   # drop anything below floor
walls.merge_vertices()
# make sure floor normals point up
fn=floor.face_normals; 
for nm,mesh in(('gamefield_floor',floor),('gamefield_walls',walls)):
    mesh.export(f'/mnt/user-data/outputs/{nm}.stl')
    print(nm,len(mesh.faces),mesh.bounds.round(1).tolist())
print('floor normals z>0 frac',(floor.face_normals[:,2]>0).mean())
