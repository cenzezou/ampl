import ampl
import numpy as np

robot = ampl.ArmBase("abb_irb6700_150_320", ampl.ArmType.Industrial6, 6)


qts = np.zeros((8, 7), dtype=np.float64)
q = np.array([[0.0]*6], dtype=np.float64)
print(robot.info())

robot.fk_qt7(q[0], qts[1:])

dir_mesh = "/home/czhou/Projects/Robotics/pamp_binding/database/robot/abb_irb6700_150_320/visual"
dir_wp = "/home/czhou/Playground"

meshes = [ampl.read_trimesh(f"{dir_mesh}/link_{i}.ply") for i in range(1, 7)]
meshes.append(ampl.read_trimesh(
    f"/home/czhou/Projects/Robotics/pamp_binding/database/tool/suction_gp12/meshes/visual/suction_rect.ply"))

for i, m in enumerate(meshes):
    v, f = m
    ampl.transform_xyz(v, v, qts[i+1])
    ampl.write_trimesh(f"{dir_wp}/{i}.ply", v, f)
