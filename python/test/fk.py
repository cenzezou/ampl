import ampl
import numpy as np

robot = ampl.ArmBase("abb_irb6700_150_320", ampl.ArmType.Industrial6, 6)


qts = np.zeros((7, 7), dtype=np.float64)
q = np.array([[0]*6], dtype=np.float64)
print(robot.info())

robot.fk_qt7(q[0], qts)
dir_wp = "/home/czhou/Projects/Robotics/pamp_binding/database/robot/abb_irb6700_150_320/collision"
meshes = [ampl.read_trimesh(f"{dir_wp}/link_{i}.ply") for i in range(1, 7)]
print(meshes[0])
