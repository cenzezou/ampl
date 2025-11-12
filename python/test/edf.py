import numpy as np
import ampl

import time, sys


def tic(print_cmd: bool = False):
    global timer_global
    timer_global = time.perf_counter_ns()
    if print_cmd:
        print("# TIC")


def toc(print_cmd: bool = True):
    global timer_global
    delta = (float(time.perf_counter_ns()) - timer_global) / 1e6
    if print_cmd:
        print(f"# TOC = {delta} MS")
    return delta


xyz_min = np.array([0.30, -0.7, 0.57 + 0.1])
xyz_max = np.array([1.70, 0.7, 0.9 + 0.1])
dx = 2.5e-3
shape = np.array((xyz_max - xyz_min) / dx, dtype=np.uint32)
print(xyz_min, dx, shape)

V_cad, F_cad = ampl.read_trimesh(
    "/home/czhou/Data/mplib/test_scene/scenes/02/configs/04/scene.ply"
)
tic()
xyz = ampl.trimesh_sampler_barycentricysplit(V_cad, F_cad, 1, sample_distance_expect=dx)
toc()
ampl.write_pointcloud(
    "/home/czhou/Data/mplib/test_scene/scenes/02/configs/04/scene_pcd_v.ply", xyz
)
# v, _, _ = ampl.read_pointcloud(
#     "/home/czhou/Data/mplib/test_scene/scenes/02/configs/04/scene_pcd.ply"
# )
edf = np.array([0] * shape.prod(), dtype=np.float32)
occ = np.zeros(shape=shape, dtype=np.uint8)
# xyz = v.astype(np.float32)
tic()
for _ in range(10):
    ampl.distancefield_xyz2edf(xyz, occ, edf, shape, xyz_min, dx, 8)
toc()

V_mc, F_mc = ampl.distancefield_df2trimesh(edf, dx, shape, xyz_min, dx)
ampl.write_trimesh(
    "/home/czhou/Data/mplib/test_scene/scenes/02/configs/04/scene_wtfs.ply", V_mc, F_mc
)
