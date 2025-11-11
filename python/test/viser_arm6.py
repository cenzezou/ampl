import time
import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf
import ampl
import trimesh


def main_arm6():
    """Main function for basic IK."""
    ROBOT_TAG = "yaskawa_gp12"
    path_urdf = (
        f"/home/czhou/Projects/Robotics/pamp_binding/database/robot/yaskawa_gp12/yaskawa_gp12.urdf"
    )
    # ROBOT_TAG="hillbot_left"
    # path_urdf= f"/home/czhou/Data/mplib/SoledadAssets-main/hillbot_beta1.0_v2/hillbot_left-link_arm.urdf"
    urdf = yourdfpy.urdf.URDF.load(path_urdf)
    dof = 6
    arm = ampl.ArmBase(ROBOT_TAG, ampl.ArmType.Industrial6, dof)
    q8 = np.zeros((8, dof), dtype=np.float64)  # 8 analytic ik solutions
    qtLink = np.zeros((dof + 1, 7), dtype=np.float64)

    q = np.array([0.0001] * 6, dtype=np.float64)  # a random pose to start

    if 1:
        mesh = trimesh.load_mesh(
            "/home/czhou/Data/A/surf_B/uv_clip_textured.obj")
        mesh.vertex_normals
        mesh.apply_scale(1e-3).apply_transform(
            trimesh.transformations.rotation_matrix(
                np.pi, [0, 1, 0], [0, 0, 0.5])
        ).apply_translation([0, 0.7, 0.5])
    which_ik = 1  # pick which ik from 8 analytic solutions
    arm.fk_links(q, qtLink)  # fk
    # get last of qLink which is tool0 (flann) frame (not link 6)
    qt_tool0 = qtLink[-1]
    tf_tool0 = ampl.qt7_to_tf44(qt_tool0)
    # iks status is uint8 = 8 bit, each bit = 0 or 1 mean solution (in)valid
    ikstatus = arm.ik(tf_tool0, q8)
    solution = q8[which_ik]

    # print(solution)

    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    whichik_handle = server.gui.add_slider(
        "#IK", initial_value=1, min=0, max=7, step=1)

    # server.scene.add_mesh_trimesh("secne", mesh)

    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base_link")
    timing_handle = server.gui.add_text(
        "1x IK Time (ms)", f"{ikstatus:08b}", disabled=True)
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=qt_tool0[4:], wxyz=qt_tool0[[3, 0, 1, 2]]
    )
    tf_tool0 = ampl.wxyz_t_to_tf44(
        np.array(ik_target.wxyz), np.array(ik_target.position)
    )

    @server.on_client_connect
    def _(client: viser.ClientHandle) -> None:

        client.camera.position = (20, 20, 20)
        client.camera.look_at = (0.0, 0, 0)
        client.camera.fov = 0.1
        # client.camera.
        # client.camera.sc
        # client.scene.add_point_cloud(
        #     "A", V, C, point_size=5e-3)

    while True:

        start_time = time.time()  # timer for solving IK.

        tf_tool0 = ampl.wxyz_t_to_tf44(
            np.array(ik_target.wxyz), np.array(ik_target.position)
        )
        ikstatus = arm.ik(tf_tool0, q8)
        solution = q8[whichik_handle.value]

        elapsed_time = time.time() - start_time
        timing_handle.value = f"{ikstatus:08b}"
        # print(solution)

        urdf_vis.update_cfg(solution)


if __name__ == "__main__":
    main_arm6()
