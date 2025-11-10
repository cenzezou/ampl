import time
import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf
import ampl,trimesh


def main():
    """Main function for basic IK."""
    ROBOT_TAG = "yaskawa_gp12"
    path_urdf=f"/home/czhou/Projects/pamp_binding/database/robot/{ROBOT_TAG}/urdf.urdf"
    ROBOT_TAG="hillbot_left"
    path_urdf= f"/home/czhou/Data/mplib/SoledadAssets-main/hillbot_beta1.0_v2/hillbot_left-link_arm.urdf"
    urdf = yourdfpy.urdf.URDF.load(path_urdf)    
    dof=7
    
    #arm = ampl.ArmBase(ROBOT_TAG, ampl.ArmType.Industrial6, 6)
    arm = ampl.ArmBase(ROBOT_TAG, ampl.ArmType.Humanoid7, 7)
    q8=np.zeros((8, dof), dtype=np.float64) # 8 analytic ik solutions
    qLink=np.zeros((dof+1, 7), dtype=np.float64) # pose of each link plus tool0 so shape = (dim(qt) x (6+1))
    
    q = np.array(
        
[ 0.46714804,  0.89479915  ,1.74232432 ,-1.27230091, -0.57711752, -0.6041509,
  0.5        

    ], dtype=np.float64)  # a random pose to start  
    
      
    #q*=0
    q8[:,-1]=q[-1]


       

    mesh = trimesh.load_mesh("/home/czhou/Data/mplib/test_scene/scenes/01/configs/05/scene.ply") 
    which_ik=5    # pick which ik from 8 analytic solutions    
    arm.fk_qt7(q,qLink)    # fk
    qt_tool0=qLink[-1] # get last of qLink which is tool0 (flann) frame (not link 6)
    tf_tool0 = ampl.qt7_to_tf44(qt_tool0)        
    ikstatus=arm.ik(tf_tool0,q8)    # iks status is uint8 = 8 bit, each bit = 0 or 1 mean solution (in)valid
    solution=q8[which_ik]

    #print(solution)



    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=3, height=3)     
    server.scene.add_mesh_trimesh("secne",mesh)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base_link")
    timing_handle = server.gui.add_number("1x IK Time (ms)", 0.001, disabled=True)    
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2,
        position=qt_tool0[4:], wxyz=qt_tool0[[3,0,1,2]]
    )
    tf_tool0 = ampl.wxyz_t_to_tf44 (np.array(ik_target.wxyz),np.array(ik_target.position))    

    while True:
        
        start_time = time.time() # timer for solving IK.
        
        tf_tool0 = ampl.wxyz_t_to_tf44 (np.array(ik_target.wxyz),np.array(ik_target.position))
        ikstatus=arm.ik(tf_tool0,q8)
        solution=q8[which_ik]
        
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)        
        #print(solution)
        
        urdf_vis.update_cfg(solution)
        


if __name__ == "__main__":
    main()