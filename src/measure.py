# measure.py
import trimesh
base = "/home/keedastro/dojourdf/src/gamefield/models/gamefield_arena/meshes/"
s = 0.001  # same scale as the SDF

f = trimesh.load(base + "gamefield_floor.stl").bounds * s
w = trimesh.load(base + "gamefield_walls.stl").bounds * s

print("floor min/max (m):", f[0], f[1])
print("walls min/max (m):", w[0], w[1])

dz = f[1][2] - w[0][2]   # lift so wall bottom == floor top
print(f"\nwalls z offset: {dz:.5f}")
print(f"walls xy offset (centre mismatch): "
      f"{((f[0]+f[1])/2 - (w[0]+w[1])/2)[:2]}")