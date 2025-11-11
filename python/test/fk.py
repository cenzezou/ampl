import ampl
import numpy as np


def test_billbot():
    dof = 7
    ROBOT_TAG = "hillbot_left"
    arm = ampl.ArmBase(ROBOT_TAG, ampl.ArmType.Humanoid7, dof)
    qLink = np.zeros((dof+1, 7), dtype=np.float64)
    q = np.array([0.1]*dof, dtype=np.float64)
    q8 = np.ones((8, dof), dtype=np.float64)  # 8 analytic ik solutions
    q8 *= 0.1
    arm.fk_links(q, qLink)
    # print(qts)

    DIR_ROBOT_DATABASE = "/home/czhou/Projects/pamp_binding/database/"
    dir_mesh = F"{DIR_ROBOT_DATABASE}/robot/hillbot_left-link_arm/convex"
    dir_wp = "/home/czhou/Playground"

    meshes = [ampl.read_trimesh(
        f"{dir_mesh}/left-link_arm_{i}.ply") for i in range(1, dof+1)]

    for i, m in enumerate(meshes):
        v, f = m
        ampl.transform_xyz(v, v, qLink[i])
        ampl.write_trimesh(f"{dir_wp}/{ROBOT_TAG}_{i}.ply", v, f)

    # get last of qLink which is tool0 (flann) frame (not link 6)
    qt_tool0 = qLink[-1]
    tf_tool0 = ampl.qt7_to_tf44(qt_tool0)
    print(tf_tool0)
    # iks status is uint8 = 8 bit, each bit = 0 or 1 mean solution (in)valid
    ikstatus = arm.ik(tf_tool0, q8)
    # solution=q8[which_ik]
    print(q8)


def test_arm6():

    import trimesh
    robot = ampl.ArmBase("yaskawa_gp12", ampl.ArmType.Industrial6, 6)
    qts = np.zeros((7, 7), dtype=np.float64)
    q = np.array([[0.0]*6], dtype=np.float64)

    tf_base = trimesh.transformations.rotation_matrix(
        np.pi/4, [0, 0, 1]).astype(np.float64)
    tf_base[2, 3] = 0.0

    robot.set_base(tf_base)

    robot.fk_links(q[0], qts)
    print(qts[:, -3:])

    q8 = np.zeros((8, 6), dtype=np.float64)
    tf_tool0 = ampl.qt7_to_tf44(qts[-1])
    iks = robot.ik(tf_tool0, q8)
    print(q8)
    # print(iks)

    DIR_ROBOT_DATABASE = "/home/czhou/Projects/pamp_binding/database/"

    dir_mesh = F"{DIR_ROBOT_DATABASE}/robot/yaskawa_gp12/visual"
    dir_wp = "/home/czhou/Playground"

    # print(tf_base)

    #

    robot.fk_links(q[0], qts)

    meshes = [ampl.read_trimesh(f"{dir_mesh}/link_{i}.ply")
              for i in range(1, 7)]
    meshes.append(ampl.read_trimesh(
        f"{DIR_ROBOT_DATABASE}/tool/suction_gp12/meshes/visual/suction_rect.ply"))

    for i, m in enumerate(meshes):
        v, f = m
        ampl.transform_xyz(v, v, qts[i])
        ampl.write_trimesh(f"{dir_wp}/{i}.ply", v, f)


if __name__ == "__main__":
    v, f = ampl.read_trimesh(
        "/home/czhou/Projects/Robotics/pamp_binding/database/robot/yaskawa_gp12/raw/gp12_base_link.stl")
    # print(v, f)
    print(len(f))
    # ampl.write_trimesh(
    #     "/home/czhou/Projects/Robotics/pamp_binding/database/robot/yaskawa_gp12/raw/gp12_base_linkaa.ply", v, f)
