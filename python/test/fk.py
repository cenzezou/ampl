import ampl
import numpy as np

import pamp


def test_billbot():
    dof=7
    ROBOT_TAG="hillbot_left"
    arm = ampl.ArmBase(ROBOT_TAG, ampl.ArmType.Humanoid7, dof)    
    qLink = np.zeros((dof+1, 7), dtype=np.float64)
    q = np.array([0.1]*dof, dtype=np.float64)
    q8=np.ones((8, dof), dtype=np.float64) # 8 analytic ik solutions
    q8*=0.1
    arm.fk_qt7(q, qLink)
    #print(qts)

    DIR_ROBOT_DATABASE= "/home/czhou/Projects/pamp_binding/database/"
    dir_mesh = F"{DIR_ROBOT_DATABASE}/robot/hillbot_left-link_arm/convex"
    dir_wp = "/home/czhou/Playground"

    meshes = [ampl.read_trimesh(f"{dir_mesh}/left-link_arm_{i}.ply") for i in range(1, dof+1)]

    for i, m in enumerate(meshes):
        v, f = m
        ampl.transform_xyz(v, v, qLink[i])
        ampl.write_trimesh(f"{dir_wp}/{ROBOT_TAG}_{i}.ply", v, f)

    qt_tool0=qLink[-1] # get last of qLink which is tool0 (flann) frame (not link 6)
    tf_tool0 = ampl.qt7_to_tf44(qt_tool0)        
    print(tf_tool0)
    ikstatus=arm.ik(tf_tool0,q8)    # iks status is uint8 = 8 bit, each bit = 0 or 1 mean solution (in)valid
    #solution=q8[which_ik]
    print(q8)


    


def test_arm6():
    robot = ampl.ArmBase("yaskawa_gp12", ampl.ArmType.Industrial6, 6)    
    qts = np.zeros((8, 7), dtype=np.float64)
    q = np.array([[0.1]*6], dtype=np.float64)
    print(robot.info())
    robot.fk_qt7(q[0], qts[1:])
    q8=np.zeros((8, 6), dtype=np.float64)
    tf_tool0=pamp.qt7_to_tf44(qts[-1])
    iks=robot.ik(tf_tool0,q8)
    print(q8)
    print(iks)
    
    DIR_ROBOT_DATABASE= "/home/czhou/Projects/pamp_binding/database/"

    dir_mesh = F"{DIR_ROBOT_DATABASE}/robot/yaskawa_gp12/visual"
    dir_wp = "/home/czhou/Playground"


    robot.fk_qt7(q8[3], qts[1:])

    meshes = [ampl.read_trimesh(f"{dir_mesh}/link_{i}.ply") for i in range(1, 7)]
    meshes.append(ampl.read_trimesh(
        f"{DIR_ROBOT_DATABASE}/tool/suction_gp12/meshes/visual/suction_rect.ply"))

    for i, m in enumerate(meshes):
        v, f = m
        ampl.transform_xyz(v, v, qts[i+1])
        ampl.write_trimesh(f"{dir_wp}/{i}.ply", v, f)



if __name__ == "__main__":
    test_billbot()        


