import time
import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf
import ampl
import trimesh


class dualarm:
    def __init__(self):
        self.urdf = [
            yourdfpy.urdf.URDF.load(
                # "/home/czhou/Data/mplib/SoledadAssets-main/hillbot_beta1.0_v2/hillbot_left-link_arm.urdf"
                "/home/czhou/Data/mplib/tianji_left/urdf/tianji_left.urdf"
                # "/home/czhou/Data/mplib/hillbot_left/urdf/hillbot_left.urdf"
            ),
            yourdfpy.urdf.URDF.load(
                f"/home/czhou/Data/mplib/tianji_right/urdf/tianji_right.urdf"
                # "/home/czhou/Data/mplib/hillbot_right/urdf/hillbot_right.urdf"
            ),
        ]
        self.dict_tf_world_base = {
            "hillbot_left": np.array(
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
                ]
            ),
            "hillbot_right": np.array(
                [
                    [
                        0.955335201910934,
                        0.2955236511323725,
                        -0.0006508541975902486,
                        0.3583128304023737,
                    ],
                    [
                        -0.0006207955070956112,
                        -0.0001955385242231009,
                        -0.9999997881887894,
                        -0.1170725392264543,
                    ],
                    [
                        -0.29552371580421943,
                        0.9553354036075901,
                        -3.3450806565744638e-06,
                        1.3100192617302837,
                    ],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
            "hillbot_right_": np.array(
                [
                    [
                        0.999999789598302,
                        3.670821584131324e-06,
                        -0.0006486831865905677,
                        0.24542413524961065,
                    ],
                    [
                        -0.0006486831730981348,
                        -3.6755870768398464e-06,
                        -0.9999997895982933,
                        -0.11699924754533196,
                    ],
                    [
                        -3.6732051033217936e-06,
                        0.9999999999865075,
                        -3.6732051032970128e-06,
                        1.4770807228841871,
                    ],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
            "hillbot_left_": np.array(
                [
                    [
                        0.999999789598302,
                        -3.6755870768151143e-06,
                        0.000648683159605702,
                        0.24557592646027523,
                    ],
                    [
                        -0.0006486831730981348,
                        -3.670821584205681e-06,
                        0.9999997895983108,
                        0.11699970322088317,
                    ],
                    [
                        -3.6732051033217936e-06,
                        -0.9999999999865075,
                        -3.673205103346574e-06,
                        1.4770798633578663,
                    ],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        }
        self.dof = 7
        self.arm = [
            ampl.ArmBase("tianji_left", ampl.ArmType.Humanoid7, self.dof),
            ampl.ArmBase("tianji_left", ampl.ArmType.Humanoid7, self.dof),
            # ampl.ArmBase("hillbot_left", ampl.ArmType.Humanoid7, self.dof),
            # ampl.ArmBase("hillbot_right", ampl.ArmType.Humanoid7, self.dof),
        ]
        self.q8 = [
            np.zeros((8, self.dof), dtype=np.float64),
            np.zeros((8, self.dof), dtype=np.float64),
        ]
        self.qtLink = [
            np.zeros((self.dof + 1, 7), dtype=np.float64),
            np.zeros((self.dof + 1, 7), dtype=np.float64),
        ]
        self.tf_world_base = [
            self.dict_tf_world_base["hillbot_left"],
            self.dict_tf_world_base["hillbot_right"],
        ]

        self.arm[0].set_base(self.tf_world_base[0])
        self.arm[1].set_base(self.tf_world_base[1])
        self.ikstatus = [np.uint8(0), np.uint8(0)]

    def ik(
        self,
        tf_world_tool0_left=None,
        tf_world_tool0_right=None,
        q_last_left=0.0,
        q_last_right=0.0,
    ):
        if tf_world_tool0_left.any():
            self.q8[0][:, -1] = q_last_left
            self.ikstatus[0] = self.arm[0].ik(tf_world_tool0_left, self.q8[0])
        if tf_world_tool0_right.any():
            self.q8[1][:, -1] = q_last_right
            self.ikstatus[1] = self.arm[1].ik(tf_world_tool0_right, self.q8[1])

    def get_ik(self, which_ik_left, which_ik_right):
        return self.q8[0][which_ik_left], self.q8[1][which_ik_right]

    def update_fk(self, q_left=None, q_right=None):
        if q_left.any():
            self.arm[0].fk_links(q_left, self.qtLink[0])
        if q_right.any():
            self.arm[1].fk_links(q_right, self.qtLink[1])

    def get_tool0_tf(self):
        return ampl.qt7_to_tf44(self.qtLink[0][-1]), ampl.qt7_to_tf44(
            self.qtLink[1][-1]
        )

    def get_tool0_qt(self):
        return self.qtLink[0][-1], self.qtLink[1][-1]


def main_dualarm7():
    q = np.array(
        [0.46714804, 0.89479915, 1.74232432, -1.27230091, -0.57711752, -0.6041509, 0.5],
        dtype=np.float64,
    )
    q *= 0.0001

    hillbot = dualarm()
    hillbot.update_fk(q, q)

    tf_tool0_left, tf_tool0_right = hillbot.get_tool0_tf()
    qt_tool0_left, qt_tool0_right = hillbot.get_tool0_qt()

    hillbot.ik(tf_tool0_left, tf_tool0_right, q[-1], q[-1])

    # print(q)
    q_left, q_right = hillbot.get_ik(5, 1)
    # print(hillbot.q8[0])
    # print(hillbot.q8[1])

    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=3, height=3)
    base_left = server.scene.add_frame("/base_link_left", show_axes=False)
    base_right = server.scene.add_frame("/base_link_right", show_axes=False)
    base_left.position = hillbot.tf_world_base[0][:3, 3]
    base_left.wxyz = trimesh.transformations.quaternion_from_matrix(
        hillbot.tf_world_base[0][:3, :3]
    )
    base_right.position = hillbot.tf_world_base[1][:3, 3]
    base_right.wxyz = trimesh.transformations.quaternion_from_matrix(
        hillbot.tf_world_base[1][:3, :3]
    )
    mesh = trimesh.load_mesh(
        "/home/czhou/Data/mplib/test_scene/scenes/01/configs/05/scene.ply"
    )
    server.scene.add_mesh_trimesh("secne", mesh)
    urdf_vis_left = ViserUrdf(server, hillbot.urdf[0], root_node_name="/base_link_left")
    urdf_vis_right = ViserUrdf(
        server, hillbot.urdf[1], root_node_name="/base_link_right"
    )
    # timing_handle = server.gui.add_number("1x IK Time (ms)", 0.001, disabled=True)
    ikstatus_handle = server.gui.add_text("IK status", f"{0:08b}", disabled=True)
    qlast_handle = server.gui.add_slider(
        "q last",
        initial_value=0,
        disabled=False,
        min=-np.pi / 2,
        max=np.pi / 2,
        step=np.pi / 64,
    )

    ik_target_left = server.scene.add_transform_controls(
        "/ik_target_left",
        scale=0.2,
        position=np.array(qt_tool0_left[4:]).flatten(),
        wxyz=qt_tool0_left[[3, 0, 1, 2]],
    )
    ik_target_right = server.scene.add_transform_controls(
        "/ik_target_right",
        scale=0.2,
        position=np.array(qt_tool0_right[4:]).flatten(),
        wxyz=qt_tool0_right[[3, 0, 1, 2]],
    )

    @server.on_client_connect
    def _(client: viser.ClientHandle) -> None:

        client.camera.position = (20, 20, 20)
        client.camera.look_at = (0.5, 0, 1.6)
        client.camera.fov = 0.1

    while True:

        start_time = time.time()  # timer for solving IK.

        tf_target_left = ampl.wxyz_t_to_tf44(
            np.array(ik_target_left.wxyz), np.array(ik_target_left.position)
        )
        tf_target_right = ampl.wxyz_t_to_tf44(
            np.array(ik_target_right.wxyz), np.array(ik_target_right.position)
        )

        tf_tool0_left = tf_target_left
        tf_tool0_right = tf_target_right
        # ikstatus = arm.ik(tf_tool0, q8)
        # solution = q8[which_ik]

        # TIANJI
        hillbot.ik(
            tf_tool0_left, tf_tool0_right, qlast_handle.value, -1 * qlast_handle.value
        )
        q_left, q_right = hillbot.get_ik(7, 6)

        # HILLBOT

        # hillbot.ik(
        #     tf_tool0_left, tf_tool0_right, qlast_handle.value, qlast_handle.value
        # )
        # q_left, q_right = hillbot.get_ik(5, 6)

        urdf_vis_left.update_cfg(q_left)
        urdf_vis_right.update_cfg(q_right)

        elapsed_time = time.time() - start_time
        ikstatus_handle.value = f"{hillbot.ikstatus[0]:08b}"
        print(f"{hillbot.ikstatus[0]:08b}", end="\r")
        # timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

    return


if __name__ == "__main__":
    main_dualarm7()
