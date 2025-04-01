"""Implement the agimus_demo_05_pick_and_place Orchestrator"""

from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pinocchio as pin
import time
import re
import pickle

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_system_default
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray


from hpp.corbaserver.manipulation import loadServerPlugin
from contact_graspnet_msgs.srv import GetSceneGrasps

from agimus_demo_05_pick_and_place.franka_gripper_client import FrankaGripperClient

from agimus_demo_05_pick_and_place.hpp_client import (
    HPPInterface,
    get_traj_points_from_path,
)
from agimus_demo_05_pick_and_place.async_subscriber import AsyncSubscriber
from agimus_demo_05_pick_and_place.trajectory_publisher import TrajectoryPublisher
from agimus_demo_05_pick_and_place.utils import multiply_poses, inverse_pose


def map_object_id(obj_id, dataset="tless"):
    num_part = obj_id.split("_")[1]
    return f"{dataset}-obj_{int(num_part):06d}"


def hardcoded_config_obj21() -> list[float]:
    """
    Place the ros2 topic echo /happypose/detections output and get a list
    """
    str_pose = """
        position:
          x: 0.10178913921117783
          y: -0.062033891677856445
          z: 0.4826200306415558
        orientation:
          x: 0.7520243449976984
          y: -0.655999015851784
          z: 0.04304297107862538
          w: 0.04766424634238109
    """
    float_values = list(map(float, re.findall(r"[-+]?\d*\.\d+|\d+", str_pose)))

    return float_values


def hardcoded_config_obj23() -> list[float]:
    """
    Place the ros2 topic echo /happypose/detections output and get a list
    """
    str_pose = """
        position:
          x: -0.06546976417303085
          y: -0.01000980008393526
          z: 0.452702134847641
        orientation:
          x: 0.7593477541545062
          y: -0.6384211079720647
          z: -0.12404735319628357
          w: -0.020536926067232224
    """
    float_values = list(map(float, re.findall(r"[-+]?\d*\.\d+|\d+", str_pose)))

    return float_values


def hardcoded_config_obj26() -> list[float]:
    """
    Place the ros2 topic echo /happypose/detections output and get a list
    """
    str_pose = """
        position:
          x: 0.18757937848567963
          y: -0.01735815778374672
          z: 0.4177013635635376
        orientation:
          x: 0.7211196097895991
          y: -0.6708606536936367
          z: -0.1536919294128396
          w: -0.0794436356995622
    """
    float_values = list(map(float, re.findall(r"[-+]?\d*\.\d+|\d+", str_pose)))

    return float_values


def get_graspnet_pose():
    return np.array(
        # best pose obj 23
        [
            [-0.6334359, 0.7723086, -0.04794085, -0.07671691],
            [-0.6801161, -0.52613074, 0.5105179, -0.06515313],
            [0.36905417, 0.3559857, 0.8585297, 0.38535887],
            [0.0, 0.0, 0.0, 1.0],
        ]
        # best pose object 20
        # [[-0.9788564,  -0.20358302 , 0.01985245, -0.11678547],
        # [ 0.09792168, -0.38117558 , 0.9193022,  -0.15793388],
        # [-0.17958704,  0.90180886 , 0.39305136,  0.44814843],
        # [ 0.     ,     0.   ,       0.   ,       1.        ]]
    )


def simulate_graspnet_output() -> dict[list[tuple[np.array, float]]]:
    fname = "/home/gepetto/ros2_ws/src/agimus-demos/agimus_demo_05_pick_and_place/graspnet_output.pkl"
    return pickle.load(open(fname, "rb"))
    # # every dict key is list of tuples  (4x4 pose (cam to grasp), score)
    # res['1_tless20'] = [
    #     (np.eye(4), 0.9),
    #     (np.eye(4), 0.8),
    #     (np.eye(4), 0.7),
    #     ]


def graspnet_to_handle(world_to_cam: pin.SE3, cam_to_grasp: pin.SE3) -> list[float]:
    # convert graspnet frame to franka hand frame
    grasp_to_ee = pin.SE3(pin.rpy.rpyToMatrix(0, 0, -np.pi / 2), np.zeros(3))
    world_to_ee = world_to_cam * cam_to_grasp * grasp_to_ee
    # from franka hand to grasp location, and align x with axis going through the fingers
    ee_to_grasp = pin.SE3(
        pin.rpy.rpyToMatrix(0, -np.pi / 2, 0), np.array([0, 0, 0.103])
    )
    handle_in_world = world_to_ee * ee_to_grasp
    return pin.SE3ToXYZQUAT(handle_in_world).tolist()


def hardcoded_config(object_name: str) -> list[float]:
    if object_name == "obj_21":
        return hardcoded_config_obj21()
    elif object_name == "obj_23":
        return hardcoded_config_obj23()
    elif object_name == "obj_26":
        return hardcoded_config_obj26()
    else:
        raise ValueError(f"Object {object_name} not found")


def posemsg2mat(pose: Pose) -> npt.NDArray:
    """Convert a ROS2 Pose message to a 4x4 numpy array."""
    return pin.XYZQUATToSE3(
        np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
    ).homogeneous


@dataclass
class OrchestratorParams:
    """Orchestrator parameters."""

    max_holding_force: float = 30.0
    parking_configuration: npt.NDArray = np.zeros(0)
    destination_configuration: npt.NDArray = np.zeros(0)


class Orchestrator(object):
    """Orchestrator of demo agimus_demo_05_pick_and_place"""

    def __init__(self):
        self._node = Node("pick_and_place")
        self.param = OrchestratorParams()

        self.franka_gripper_cient = FrankaGripperClient(self._node)
        self.default_object_name = "obj_23"
        self.use_hardcoded_poses = True
        self.run_in_sim = False

        self.trajectory_publisher = TrajectoryPublisher(self._node)

        self.state_client = AsyncSubscriber(
            self._node,
            JointState,
            "/joint_states",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.target_client = AsyncSubscriber(
            self._node,
            Pose,
            "/target_object",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        if not self.use_hardcoded_poses:
            self.vision_client = AsyncSubscriber(
                self._node,
                Detection2DArray,
                "/happypose/detections",
                qos_profile_system_default,
            )

        self.grasps_client = self._node.create_client(
            GetSceneGrasps, "contact_graspnet/get_scene_grasps"
        )

        self.open_gripper()
        loadServerPlugin("corbaserver", "manipulation-corba.so")
        loadServerPlugin("corbaserver", "bin_picking.so")
        # self.detected_grasps = simulate_graspnet_output()
        self.detected_grasps = self.get_all_grasps()

    def get_all_grasps(self) -> dict[list[tuple[np.array, float]]]:
        """Get all grasps from the graspnet service"""
        self.grasps_client.wait_for_service()
        self._node.get_logger().info("Graspnet service is available, calling...")
        request = GetSceneGrasps.Request()
        future = self.grasps_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        self._node.get_logger().info("Graspnet service response received!")
        resp: GetSceneGrasps.GetSceneGrasps.Response = future.result()
        scene_grasps = resp.scene_grasps

        object_nb = len(scene_grasps.object_types)
        all_grasps = {}
        for i in range(object_nb):
            object_type = scene_grasps.object_types[i]
            object_id = f"{i}_{object_type}"

            # grasps of the i-th object
            grasps_i = scene_grasps.object_grasps[i]

            all_grasps[object_id] = [
                (posemsg2mat(grasp), score)
                for grasp, score in zip(grasps_i.grasps, grasps_i.scores)
            ]

        return all_grasps

    def select_object_to_pick(self) -> list[tuple[np.array, float]]:
        """The first object to pick is the one that is the closest to the camera on z-axis"""
        closest_dist = np.inf
        object_to_pick = None
        for k, v in self.detected_grasps.items():
            print(f"Object {k} has {len(v)} grasps")
            # print(f"Grasps {v}")
            for grasp, _ in v:
                if grasp[2, 3] < closest_dist:
                    closest_dist = grasp[2, 3]
                    object_to_pick = k
        print("Picking up object", object_to_pick, "with distance", closest_dist)
        return object_to_pick

    def get_most_confident_object_pose(
        self, detection_msg: Detection2DArray, object_name: str
    ) -> list[float]:
        # TODO: change the map if we want to use YCBV
        filtered_detections = [
            (d, d.results[0].hypothesis.score)
            for d in detection_msg.detections
            if d.results[0].hypothesis.class_id == map_object_id(object_name)
        ]
        if len(filtered_detections) == 0:
            return
        detection = max(filtered_detections, key=lambda pair: pair[1])[0]
        pose: Pose = detection.results[0].pose.pose
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def open_gripper(self):
        self.franka_gripper_cient.send_goal(position=0.0385, max_effort=10.0)
        # TODO: change it to something normal
        time.sleep(0.05)

    def close_gripper(self):
        self.franka_gripper_cient.send_goal(
            position=0.0, max_effort=self.param.max_holding_force
        )
        # TODO: change it to something normal
        time.sleep(0.05)

    def grasp(self):
        self.franka_gripper_cient.grasp()
        # TODO: change it to something normal
        time.sleep(1.0)

    def publish(self, path_vector):
        traj = get_traj_points_from_path(path_vector)
        # TODO: get this from OCP params somehow
        traj += [traj[-1]] * 40  # OCP horizon
        self.trajectory_publisher.publish(traj)

    def go_to(self, desired_configuration):
        self.hpp_client = HPPInterface(
            object_name=self.default_object_name, use_spline_gradient_based_opt=False
        )
        current_robot_state = self.state_client.wait_for_future()
        backup_goal_pose = self.hpp_client.goal_obj_pose.copy()
        self.hpp_client.goal_obj_pose = self.hpp_client.start_obj_pose.copy()
        traj = self.hpp_client.plan(
            list(current_robot_state.position), desired_configuration
        )
        self.publish(traj)
        # Commented out since restart does not work properly (corba crashes)
        # self.hpp_client.restart()
        self.hpp_client.goal_obj_pose = backup_goal_pose.copy()
        # del self.hpp_client

    def pick_and_place(self, object_name: str, return_to_init=True):
        self.hpp_client = HPPInterface(
            object_name=object_name, use_spline_gradient_based_opt=False
        )
        current_robot_state = self.state_client.wait_for_future()
        if self.run_in_sim:
            hardcoded_joint_position = [
                0.360436581289559,
                -0.8372071957341715,
                -1.0557778702195493,
                -2.613699715701293,
                -0.14910513448717685,
                2.0829864285257123,
                -0.26239278887382195,
                0.035,
                0.035,
            ]
        robot_q = (
            hardcoded_joint_position
            if self.run_in_sim
            else list(current_robot_state.position)
        )
        hpp_q_init = (
            robot_q
            + self.hpp_client.start_obj_pose
            + self.hpp_client.default_obstacle_pose
        )
        self.hpp_client.robot.setCurrentConfig(hpp_q_init)
        # TODO: change from hardcoded robot name
        cam_in_world_pose = self.hpp_client.robot.getLinkPosition(
            linkName="panda/camera_color_optical_frame"
        )
        if self.use_hardcoded_poses:
            # TEMP fix: just hardcode pose from happypose

            handles_to_add = []
            if object_name == "default_obj":
                object_to_pick = self.select_object_to_pick()
                possible_grasps = self.detected_grasps[object_to_pick]
                # sort and leave only 20 best grasps
                possible_grasps = sorted(
                    possible_grasps, key=lambda x: x[1], reverse=True
                )[:10]
                print(f"object {object_to_pick} has {len(possible_grasps)} grasps")
                # take the first grasp as identity and place the object there
                # identity handles are defined in the srdf file
                # express other handles in this frame
                ref_grasp_pose, _ = possible_grasps[0]
                obj_in_world_pose = graspnet_to_handle(
                    pin.XYZQUATToSE3(cam_in_world_pose), pin.SE3(ref_grasp_pose)
                )
                for grasp, _ in possible_grasps[1:]:
                    handle_in_world_pose = graspnet_to_handle(
                        pin.XYZQUATToSE3(cam_in_world_pose), pin.SE3(grasp)
                    )
                    # convert to be expressed in the object frame

                    handles_to_add.append(
                        multiply_poses(
                            inverse_pose(obj_in_world_pose), handle_in_world_pose
                        )
                    )
                # rotate around x np.pi to account for both possible orientations of gripper
                # handle_in_world_pose = multiply_poses(
                #     handle_in_world_pose, [0, 0, 0, 1, 0, 0, 0]
                # )
                # handles_to_add.append([0.0, 0.0, 0.0] + handle_in_world_pose[3:])
                self.hpp_client.add_handles(handles_to_add)
            else:
                obj_in_cam_pose = hardcoded_config(object_name)
                if obj_in_cam_pose is None:
                    raise ValueError(f"No {object_name} object detected")
                obj_in_world_pose = multiply_poses(cam_in_world_pose, obj_in_cam_pose)

        else:
            # REAL setup, TODO: fix communication error when happy pose is running
            print("waiting for obj pose")
            object_detections = self.vision_client.wait_for_future()
            print("got obj pose")
            obj_in_cam_pose = self.get_most_confident_object_pose(
                object_detections, object_name
            )

        # TODO: make this better
        obj_in_world_pose[3:] = obj_in_world_pose[3:] / np.linalg.norm(
            obj_in_world_pose[3:]
        )
        self.hpp_client.start_obj_pose = list(obj_in_world_pose)
        hpp_q_init = (
            robot_q
            + self.hpp_client.start_obj_pose
            + self.hpp_client.default_obstacle_pose
        )
        self.hpp_client.robot.setCurrentConfig(hpp_q_init)
        grasp_path, placing_path, freefly_path = self.hpp_client.plan(
            list(current_robot_state.position)
        )
        # self.go_to([
        #     0.2675230724769726,
        #     -0.3003997491702699,
        #     0.062178244123872704,
        #     -2.185557396537362,
        #     -0.12250506031051378,
        #     2.1027184269693158,
        #     1.2193136738787091,
        #     0.03,
        #     0.03
        # ])
        # self.open_gripper()
        self.open_gripper()
        self.publish(grasp_path)
        if placing_path is not None:
            # TODO: check automatically
            if self.run_in_sim:
                self.close_gripper()  # for simulation
            else:
                self.grasp()  # for hardware robot
            self.publish(placing_path)
            self.open_gripper()
            if return_to_init:
                self.publish(freefly_path)
        if object_name == "default_obj":
            self.detected_grasps.pop(object_to_pick)
        # Commented out since restart does not work properly (corba crashes)
        # self.hpp_client.restart()
        # del self.hpp_client

    def pick_and_place_all(self):
        while len(self.detected_grasps) > 0:
            print("Picking the object")
            self.pick_and_place("default_obj", return_to_init=False)
            time.sleep(1.0)  # update if your computer is strong

    # def go_to_ee(self, target_ee):
    #     current_robot_state = self.state_client.wait_for_new_state()
    #     trajectory = self.hpp_client.plan_ee(current_robot_state.position, target_ee)
    #     self.trajectory_publisher.publish(trajectory)

    # def go_to_parking_pose(self):
    #     self.go_to(self.param.parking_configuration)

    # def go_to_destination_pose(self):
    #     self.go_to(self.param.destination_configuration)

    # def go_to_pre_grasp(self):
    #     new_target_pose = self.target_client.wait_for_new_target_pose()
    #     trajectory = self.go_to_ee(new_target_pose)
    #     self.trajectory_publisher(trajectory)
