from pathlib import Path
from hpp.rostools import process_xacro
import pinocchio as pin

def generate_xacro_save_to_file(xacro_path):
    urdf_string = process_xacro(xacro_path)
    urdf_path = xacro_path + 'parsed.urdf' 
    with open(urdf_path, 'w') as file:
        file.write(urdf_string)
    print(f"String has been saved to {urdf_path}")
    return urdf_path

def load_models_datas(xacro_path):
    urdf_path = generate_xacro_save_to_file(xacro_path)
    m, col_m, vis_m = pin.buildModelsFromUrdf(urdf_path, package_dirs=Path(__file__).parent)
    d, c_d, v_d = pin.createDatas(m, col_m, vis_m)
    return m, col_m, vis_m, d, c_d, v_d 


package_location = Path(__file__).parent
xacro_path1 = str(package_location / "urdf/demo.urdf.xacro")
xacro_path2 = "/home/gepetto/ros2_ws/src/vcs_franka/franka_description/robots/fer/fer.urdf.xacro"

m1, cm1, vm1, d1, cd1, vd1 = load_models_datas(xacro_path1)
m2, cm2, vm2, d2, cd2, vd2 = load_models_datas(xacro_path2)


q = pin.randomConfiguration(m1)

pin.forwardKinematics(m1, d1, q)
pin.updateGeometryPlacements(m1, d1, cm1, cd1)
pin.forwardKinematics(m2, d2, q)
pin.updateGeometryPlacements(m2, d2, cm2, cd2)

for g in cm1.geometryObjects:
    print(g.name)
    print(g.meshPath)
    if g.meshPath == 'BOX':
        print(g.geometry.halfSide)

for g in cm2.geometryObjects:
    print(g.name)
    print(g.meshPath)
    if g.meshPath == 'BOX':
        print(g.geometry.halfSide)




# urdf_string = process_xacro(str(package_location / "urdf/demo.urdf.xacro"))
# # robot = pin.RobotWrapper.BuildFromURDF(urdf_string)

# urdf_path_demo = str(package_location / "demo_parsed.urdf")
# with open(urdf_path_demo, 'w') as file:
#     file.write(urdf_string)
# print(f"String has been saved to {urdf_path_demo}")

# model1, collision_model1, visual_model1 = pin.buildModelsFromUrdf(urdf_path_demo, package_dirs=[package_location, ])
# print([f.name for f in  model1.frames])
# # for frame_id, frame in model.frames.items():
# #     print(f"Link ID: {frame_id}, Name: {frame.name}")

# print(pin.neutral(model1))
# # /home/gepetto/ros2_ws/src/vcs_franka/franka_description/robots/fer/fer.urdf.xacro

# package_location = Path(__file__).parent
# urdf_string = process_xacro("/home/gepetto/ros2_ws/src/vcs_franka/franka_description/robots/fer/fer.urdf.xacro")
# urdf_path_franka = str(package_location / "demo_parsed2.urdf")
# with open(urdf_path_franka, 'w') as file:
#     file.write(urdf_string)
# print(f"String has been saved to {urdf_path_franka}")

# model2, collision_model2, visual_model2 = pin.buildModelsFromUrdf(urdf_path_franka, package_dirs=[package_location, ])
# data2, collision_data2, visual_data2 = pin.createDatas(
#     model2, collision_model2, visual_model2 
# )
/home/gepetto/ros2_ws/install/share/franka_description/meshes/robot_arms/fer/collision/link0.stl

# q = pin.randomConfiguration(model2)
# print(f"q: {q.T}")
 
# # Perform the forward kinematics over the kinematic tree
# pin.forwardKinematics(model2, data2, q)
 
# # Update Geometry models
# pin.updateGeometryPlacements(model2, data2, collision_model2, collision_data2)

 

# print([f.name for f in  model2.frames])

# print(pin.neutral(model2))
# frame_id = model2.getFrameId('fer_link0')
# frame = model2.frames[frame_id]
# print(frame.name)
# print(frame.placement)  # in parent joint
# print(frame.parentJoint)
# # parent_joint = model2.joints[frame.parentJoint]
# # print(parent_joint)
# # print(frame.parentFrame)
# # print(frame.type)
# # print(len(frame.shapes))
# # print(len(frame.shapes))
# # print(isinstance(frame.shapes[0], pin.Capsule))
# # print(isinstance(frame.shapes[0], pin.Mesh))

# # for i in range(9):
# #     print(f"Comparing {i} link")
# print("Ngeoms", collision_model2.ngeoms)
# geometry_id = collision_model2.getGeometryId('fer_link0')
# print(geometry_id)
# for g in collision_model2.geometryObjects:
#     print(g.name)
#     print(g.meshPath)
#     if g.meshPath == 'BOX':
#         print(g.geometry.halfSide)

# # geometry = collision_model2.geometryObjects[1]
# # if isinstance(geometry, pin.Capsule):
# #     print(f"Shape is Capsule with radius: {geometry.radius}, length: {geometry.length}")
# # elif isinstance(geometry, pin.Mesh):
# #     print(f"Shape is Mesh with name: {geometry.filename}")
# # else:
# #     print("Unknown shape type")
    

# #     if len(frame.shapes) > 0 and isinstance(frame.shapes[0], pinocchio.Capsule):
# #         capsule = frame.shapes[0]
# #         print(f"  Shape is Capsule with radius: {capsule.radius}, length: {capsule.length}")
    
# #     # Check if the shape is a Mesh
# #     elif len(frame.shapes) > 0 and isinstance(frame.shapes[0], pinocchio.Mesh):
# #         mesh = frame.shapes[0]
# #         print(f"  Shape is Mesh with name: {mesh.name}")


panda_leftfinger_0
BOX
[0.011  0.0075 0.01  ]
panda_leftfinger_1
BOX
[0.011  0.0044 0.0019]
panda_leftfinger_2
BOX
[0.00875 0.0035  0.01175]
panda_leftfinger_3
BOX
[0.00875 0.0076  0.00925]
panda_rightfinger_0
BOX
[0.011  0.0075 0.01  ]
panda_rightfinger_1
BOX
[0.011  0.0044 0.0019]
panda_rightfinger_2
BOX
[0.00875 0.0035  0.01175]
panda_rightfinger_3
BOX
[0.00875 0.0076  0.00925]