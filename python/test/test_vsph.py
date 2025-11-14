import numpy as np


import ampl


sphs = ampl.VSphG8f()

xyzr = np.array(
    [
        [0, 0, 0, 1],
        [1, 2, 3, 2],
        [3, 4, 1, 2],
        [0, 0, 0, 1],
        [1, 2, 3, 2],
        [3, 4, 1, 2],
        [0, 0, 0, 1],
        [1, 2, 3, 2],
        [3, 4, 1, 2],
    ],
    dtype=np.float32,
)
offset = np.array([3, 6], dtype=np.uint32)

xyzr = [
    np.loadtxt(
        f"/home/czhou/Projects/pamp_binding/database/robot/hillbot_left-link_arm/sphere/2/left-link_arm_{i}.txt",
        dtype=np.float32,
    ).reshape((-1, 4))
    for i in range(1, 8)
]
offset = [len(a) for a in xyzr]


xyzr = np.vstack(xyzr)
offset = np.array(offset, dtype=np.uint32)
print(offset)
print(xyzr)
print(offset)
ampl.collision_initialize_object(xyzr, offset, sphs)
# print(sphs.nb_sph)
# print(sphs.nb_offset)
