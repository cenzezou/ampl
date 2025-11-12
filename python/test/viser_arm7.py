import time
import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf
import ampl
import trimesh


def main_arm7():
    """Main function for basic IK."""
    # ROBOT_TAG = "tianji_left"
    # path_urdf = f"/home/czhou/Data/mplib/tianji/urdf/urdf.urdf"
    ROBOT_TAG = "hillbot_left"
    path_urdf = f"/home/czhou/Data/mplib/SoledadAssets-main/hillbot_beta1.0_v2/hillbot_left-link_arm.urdf"
    urdf = yourdfpy.urdf.URDF.load(path_urdf)
    dof = 7
    arm = ampl.ArmBase(ROBOT_TAG, ampl.ArmType.Humanoid7, dof)
    q8 = np.zeros((8, dof), dtype=np.float64)  # 8 analytic ik solutions
    qtLink = np.zeros((dof + 1, 7), dtype=np.float64)

    tf_world_base = np.array(
        [
            [
                0.955335201910934,
                -0.2955236559058397,
                0.000648683159605702,
                0.35846462161303827,
            ],
            [
                -0.0006207955070956112,
                0.00018819211556723273,
                0.9999997895983108,
                0.11692641153976084,
            ],
            [
                -0.29552371580421943,
                -0.9553354036063848,
                -3.673205103346574e-06,
                1.310018402203963,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    q = np.array([0.0001] * dof, dtype=np.float64)  # a random pose to start
    arm.set_base(tf_world_base)
    if 0:
        mesh = trimesh.load_mesh("/home/czhou/Data/A/surf_B/uv_clip_textured.obj")
        mesh.vertex_normals
        mesh.apply_scale(1e-3).apply_transform(
            trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0], [0, 0, 0.5])
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
    whichik_handle = server.gui.add_slider("#IK", initial_value=1, min=0, max=7, step=1)
    qlast_handle = server.gui.add_slider(
        "q last", initial_value=5, disabled=False, min=-1.5, max=1.5, step=0.05
    )
    # server.scene.add_mesh_trimesh("secne", mesh)
    robot_base = server.scene.add_frame("/base_link", show_axes=False)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base_link")
    timing_handle = server.gui.add_text(
        "1x IK Time (ms)", f"{ikstatus:08b}", disabled=True
    )
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=qt_tool0[4:], wxyz=qt_tool0[[3, 0, 1, 2]]
    )
    tf_tool0 = ampl.wxyz_t_to_tf44(
        np.array(ik_target.wxyz), np.array(ik_target.position)
    )

    robot_base.position = tf_world_base[:3, 3]  # Move to (x=1, y=0, z=0.5).
    robot_base.wxyz = trimesh.transformations.quaternion_from_matrix(
        tf_world_base[:3, :3]
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

    q_search = np.linspace(-1.5, 1.5, num=50, endpoint=True)
    q_best = q.copy()
    ikstatus_best = ikstatus
    import copy

    while True:

        start_time = time.time()  # timer for solving IK.

        tf_tool0 = ampl.wxyz_t_to_tf44(
            np.array(ik_target.wxyz), np.array(ik_target.position)
        )
        q3z_best = 1000
        for ql in q_search:
            q8[:, -1] = ql
            ikstatus = arm.ik(tf_tool0, q8)
            if (ikstatus >> whichik_handle.value) & 1:
                arm.fk_links(q8[whichik_handle.value], qtLink)
                q3z = qtLink[3, -1]
                # print(qtLink[3, -1])
                if q3z < q3z_best:
                    q3z_best = q3z
                    q_best = q8[whichik_handle.value].copy()
                    ikstatus_best = copy.copy(ikstatus)

        # print(q3z_best)
        qlast_handle.value = q_best[-1]
        solution = q_best
        # print(solution[1])
        # q8[:, -1] = qlast_handle.value
        # ikstatus = arm.ik(tf_tool0, q8)
        # solution = q8[whichik_handle.value]

        # arm.fk_links(solution, qtLink)
        # print(qtLink[3, -1])

        elapsed_time = time.time() - start_time
        timing_handle.value = f"{ikstatus_best:08b}"
        # print(solution)
        urdf_vis.update_cfg(solution)


if __name__ == "__main__":
    main_arm7()
