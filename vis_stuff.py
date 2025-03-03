import numpy as np
from robomeshcat import Robot, Scene, Object
import pinocchio as pin

scene = Scene()


"Create the first robot and add it to the scene"
urdf_path = (
    "/home/ros/sandbox_agimus/agimus_dev_container/agimus-demos/demo_parsed.urdf"
)
mesh_folder_path = "/home/ros/sandbox_agimus/agimus_dev_container/vcs_franka"
rob = Robot(urdf_path=urdf_path, mesh_folder_path=mesh_folder_path, name="visual")
scene.add_robot(rob)

# ycbv
ycbv = Object.create_mesh(
    path_to_mesh="/home/ros/sandbox_agimus/agimus_dev_container/agimus-demos/agimus_demo_05_pick_and_place/agimus_demo_05_pick_and_place/urdf/obj_23.ply",
    scale=1e-3,
    name="ycbv",
)
scene.add_object(ycbv)
q_list = np.load(
    "/home/ros/sandbox_agimus/agimus_dev_container/agimus-demos/agimus_demo_05_pick_and_place/agimus_demo_05_pick_and_place/three_q.npy",
    allow_pickle=True,
)
print(q_list.shape)

with scene.animation(fps=1):  # start the animation with the current scene
    scene.render()
    for q in q_list:
        rob[:] = q[:9]
        ycbv.pose = pin.XYZQUATToSE3(q[9 : 9 + 7]).homogeneous
        scene.render()
scene.render_image()
