# measure.py — derive the gamefield floor/wall offsets from the arena meshes.
#
# Usage (from the repository root, with the same venv/ROS env that provides
# trimesh):
#     python3 src/measure.py
from pathlib import Path

import trimesh

base = Path(__file__).resolve().parent / "gamefield" / "models" / "gamefield_arena" / "meshes"
s = 0.001  # same scale as the SDF

f = trimesh.load(str(base / "gamefield_floor.stl")).bounds * s
w = trimesh.load(str(base / "gamefield_walls.stl")).bounds * s

print("floor min/max (m):", f[0], f[1])
print("walls min/max (m):", w[0], w[1])

dz = f[1][2] - w[0][2]   # lift so wall bottom == floor top
print(f"\nwalls z offset: {dz:.5f}")
print(f"walls xy offset (centre mismatch): "
      f"{((f[0]+f[1])/2 - (w[0]+w[1])/2)[:2]}")